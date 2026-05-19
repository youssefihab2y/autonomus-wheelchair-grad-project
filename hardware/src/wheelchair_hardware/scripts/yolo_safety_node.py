#!/usr/bin/env python3
"""
YOLO Safety + Follow-Me Node

Two modes selected by publishing to /follow_mode (std_msgs/Bool):

  FALSE (default) — SAFETY mode
    Nav2 → /cmd_vel → [safety filter] → /cmd_vel_safe → ESP32
    Camera detects obstacles and slows/stops the wheelchair automatically.

  TRUE — FOLLOW-ME mode
    Nav2 is ignored. Wheelchair follows the nearest person in camera view,
    keeping a comfortable distance (~1.2 m). Like a smart companion.

    Algorithm:
      1. Find the largest person bounding box (= closest person)
      2. Turn toward them using horizontal box-center error
      3. Move forward/back to maintain target distance
      4. Stop if no person visible for > lost_timeout_s seconds

Safety zones (both modes):
  CLEAR   > 2.5 m  — full speed / normal following
  CAUTION 1.0–2.5 m — reduced speed
  DANGER  < 1.0 m  — full stop

Subscribes
  /camera/image            — RGB camera feed
  /cmd_vel                 — Nav2 velocity (used in safety mode only)
  /follow_mode             — std_msgs/Bool  true=follow-me  false=safety

Publishes
  /cmd_vel_safe            — output velocity → ESP32 bridge
  /yolo/image/compressed   — annotated JPEG → web dashboard camera
  /yolo/safety             — std_msgs/String  "clear"/"caution"/"danger"
  /yolo/distance           — std_msgs/Float32 nearest person/obstacle (m)
  /yolo/follow_status      — std_msgs/String  follow-me status string
"""

import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Bool

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    _CV_OK = True
except ImportError:
    _CV_OK = False

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False


# ── COCO class table ──────────────────────────────────────────────────────────
_CLASSES = {
    0:  ('person',        1.0),
    16: ('dog',           0.8),
    17: ('cat',           0.5),
    24: ('backpack',      0.4),
    28: ('suitcase',      0.6),
    56: ('chair',         0.7),
    57: ('couch',         0.7),
    58: ('potted plant',  0.5),
    59: ('bed',           0.8),
    60: ('dining table',  0.7),
    62: ('tv',            0.5),
    72: ('refrigerator',  0.8),
}

_HEIGHT = {
    0: 1.70, 16: 0.55, 17: 0.25, 56: 0.90, 57: 0.75,
    59: 0.55, 60: 0.75, 72: 1.70, 28: 0.60,
}
_DEFAULT_HEIGHT = 0.60

_ZONE_COLOR = {
    'clear':   ( 50, 200,  50),
    'caution': (  0, 165, 255),
    'danger':  ( 30,  30, 255),
}


class YoloSafetyNode(Node):

    def __init__(self):
        super().__init__('yolo_safety_node')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('model_path',           '')
        self.declare_parameter('device',               'cpu')   # 'cpu' or '0' for GPU
        self.declare_parameter('camera_topic',         '/camera/image')
        self.declare_parameter('confidence',           0.45)
        self.declare_parameter('danger_distance_m',    1.0)
        self.declare_parameter('caution_distance_m',   2.5)
        self.declare_parameter('speed_scale_caution',  0.40)
        self.declare_parameter('frame_skip',           2)
        self.declare_parameter('focal_length_px',      600.0)
        self.declare_parameter('camera_timeout_s',     3.0)
        self.declare_parameter('show_window',          False)
        # follow-me specific
        self.declare_parameter('follow_target_dist_m', 1.2)   # ideal gap to person
        self.declare_parameter('follow_max_speed',     0.12)  # m/s forward max
        self.declare_parameter('follow_turn_gain',     0.6)   # angular response
        self.declare_parameter('follow_lost_timeout_s',2.0)   # stop if no person

        p = lambda n: self.get_parameter(n).value
        self._device        = p('device')
        self._conf          = p('confidence')
        self._danger_d      = p('danger_distance_m')
        self._caution_d     = p('caution_distance_m')
        self._spd_caution   = p('speed_scale_caution')
        self._skip          = p('frame_skip')
        self._focal         = p('focal_length_px')
        self._cam_timeout   = p('camera_timeout_s')
        self._show_win      = p('show_window')
        self._follow_dist   = p('follow_target_dist_m')
        self._follow_speed  = p('follow_max_speed')
        self._turn_gain     = p('follow_turn_gain')
        self._lost_timeout  = p('follow_lost_timeout_s')

        # ── load YOLO ─────────────────────────────────────────────────────
        self._model  = None
        self._bridge = CvBridge() if _CV_OK else None

        if not _CV_OK:
            self.get_logger().warn('cv2 not available — pass-through mode')
        elif not _YOLO_OK:
            self.get_logger().warn('ultralytics not installed — pass-through mode\n'
                                   '  pip install ultralytics')
        else:
            mp = p('model_path') or self._find_model()
            if mp and os.path.exists(mp):
                self.get_logger().info(f'Loading YOLO: {mp}  device={self._device}')
                self._model = YOLO(mp)
                self._model(np.zeros((64, 64, 3), dtype=np.uint8),
                            conf=self._conf, device=self._device, verbose=False)
                self.get_logger().info('YOLO ready ✓')
            else:
                self.get_logger().warn(f'Model not found ({mp}) — pass-through mode')

        # ── shared state ──────────────────────────────────────────────────
        self._lock         = threading.Lock()

        # safety mode state
        self._safety       = 'clear'
        self._nearest_m    = 99.0
        self._danger_hits  = 0
        self._clear_hits   = 0

        # follow-me state
        self._follow_mode  = False          # toggled by /follow_mode topic
        self._person_x_err = 0.0           # -1..+1 horizontal error
        self._person_dist  = 99.0          # estimated distance to target
        self._person_seen  = False
        self._last_seen_t  = 0.0           # time of last person detection

        # general
        self._last_cam_t   = 0.0
        self._frame_cnt    = 0

        # ── inference queue ───────────────────────────────────────────────
        self._q = queue.Queue(maxsize=1)
        if self._model:
            threading.Thread(
                target=self._infer_loop, daemon=True, name='yolo-infer'
            ).start()

        # ── publishers ────────────────────────────────────────────────────
        self._vel_pub    = self.create_publisher(Twist,           '/cmd_vel_safe',        10)
        self._lvl_pub    = self.create_publisher(String,          '/yolo/safety',          5)
        self._dst_pub    = self.create_publisher(Float32,         '/yolo/distance',        5)
        self._img_pub    = self.create_publisher(CompressedImage, '/yolo/image/compressed',1)
        self._fol_pub    = self.create_publisher(String,          '/yolo/follow_status',   5)

        # ── subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Image, p('camera_topic'), self._cam_cb,    1)
        self.create_subscription(Twist, '/cmd_vel',         self._vel_cb,  10)
        self.create_subscription(Bool,  '/follow_mode',     self._mode_cb, 10)

        # follow-me control loop at 10 Hz (runs always, outputs only if active)
        self.create_timer(0.1, self._follow_tick)

        self.create_timer(5.0, self._diag)

        self.get_logger().info(
            f'YOLO node ready  [{"ACTIVE" if self._model else "PASS-THROUGH"}]\n'
            f'  Safety: danger<{self._danger_d}m  caution<{self._caution_d}m\n'
            f'  Follow-Me: target={self._follow_dist}m  max={self._follow_speed}m/s'
        )

    # ── mode toggle ───────────────────────────────────────────────────────────

    def _mode_cb(self, msg: Bool):
        with self._lock:
            prev = self._follow_mode
            self._follow_mode = msg.data
        if msg.data != prev:
            mode = 'FOLLOW-ME' if msg.data else 'SAFETY'
            self.get_logger().info(f'Mode switched → {mode}')
            if msg.data:
                # reset follow state
                with self._lock:
                    self._last_seen_t = 0.0
                    self._person_seen = False

    # ── camera callback ───────────────────────────────────────────────────────

    def _cam_cb(self, msg: Image):
        self._last_cam_t = time.time()
        self._frame_cnt += 1
        if self._model is None:
            return
        if self._frame_cnt % self._skip != 0:
            return
        if not self._q.empty():
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
        try:
            self._q.put_nowait(msg)
        except queue.Full:
            pass

    # ── inference thread ──────────────────────────────────────────────────────

    def _infer_loop(self):
        while rclpy.ok():
            try:
                msg = self._q.get(timeout=1.0)
            except queue.Empty:
                continue

            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().warn(f'cv_bridge: {e}')
                continue

            h, w = frame.shape[:2]

            try:
                results = self._model(frame, conf=self._conf, device=self._device, verbose=False)
            except Exception as e:
                self.get_logger().warn(f'YOLO: {e}')
                continue

            canvas    = frame.copy()
            nearest   = 99.0
            new_level = 'clear'

            # best person for follow-me (largest box = closest)
            best_person_area = 0
            best_person      = None   # (cx_norm, dist_m)

            boxes = results[0].boxes
            if boxes is not None and len(boxes):
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf   = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                    name, weight = _CLASSES.get(
                        cls_id, (self._model.names.get(cls_id, '?'), 0.3)
                    )
                    real_h = _HEIGHT.get(cls_id, _DEFAULT_HEIGHT)
                    px_h   = max(y2 - y1, 1)
                    dist   = (real_h * self._focal) / px_h

                    if dist < nearest:
                        nearest = dist

                    # safety zone
                    if   dist < self._danger_d:  zone = 'danger'
                    elif dist < self._caution_d: zone = 'caution'
                    else:                         zone = 'clear'

                    if weight >= 0.5:
                        if zone == 'danger':
                            new_level = 'danger'
                        elif zone == 'caution' and new_level == 'clear':
                            new_level = 'caution'

                    # track best person for follow-me
                    if cls_id == 0:
                        area = (x2 - x1) * (y2 - y1)
                        if area > best_person_area:
                            best_person_area = area
                            cx_norm = ((x1 + x2) / 2.0 - w / 2.0) / (w / 2.0)
                            best_person = (cx_norm, dist)

                    # draw box
                    col = _ZONE_COLOR[zone]
                    cv2.rectangle(canvas, (x1, y1), (x2, y2), col, 2)
                    lbl = f'{name} {dist:.1f}m {conf:.0%}'
                    cv2.putText(canvas, lbl, (x1, max(22, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 2)

            # ── update safety state (debounced) ───────────────────────────
            with self._lock:
                if new_level == 'danger':
                    self._danger_hits += 1
                    self._clear_hits   = 0
                    if self._danger_hits >= 2:
                        self._safety = 'danger'
                elif new_level == 'caution':
                    self._danger_hits  = 0
                    self._clear_hits   = 0
                    self._safety = 'caution'
                else:
                    self._danger_hits  = 0
                    self._clear_hits  += 1
                    if self._clear_hits >= 4:
                        self._safety = 'clear'
                self._nearest_m = nearest

                # update follow-me person state
                if best_person is not None:
                    self._person_x_err = best_person[0]
                    self._person_dist  = best_person[1]
                    self._person_seen  = True
                    self._last_seen_t  = time.time()
                else:
                    self._person_seen  = False

                lvl = self._safety
                fm  = self._follow_mode

            # ── HUD ───────────────────────────────────────────────────────
            col = _ZONE_COLOR[lvl]
            mode_str = 'FOLLOW-ME' if fm else 'SAFETY'
            cv2.rectangle(canvas, (0, 0), (w, 38), (10, 10, 10), -1)
            cv2.putText(canvas,
                        f'  [{mode_str}]  {lvl.upper()}  nearest:{nearest:.1f}m',
                        (8, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.70, col, 2)

            # ── publish compressed image ───────────────────────────────────
            try:
                ok, enc = cv2.imencode('.jpg', canvas,
                                       [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    cimg = CompressedImage()
                    cimg.header = msg.header
                    cimg.format = 'jpeg'
                    cimg.data   = enc.tobytes()
                    self._img_pub.publish(cimg)
            except Exception:
                pass

            s = String();  s.data = lvl;           self._lvl_pub.publish(s)
            d = Float32(); d.data = float(nearest); self._dst_pub.publish(d)

            if self._show_win:
                cv2.imshow('YOLO', canvas)
                cv2.waitKey(1)

    # ── follow-me control loop (10 Hz timer) ──────────────────────────────────

    def _follow_tick(self):
        with self._lock:
            fm        = self._follow_mode
            seen      = self._person_seen
            last_seen = self._last_seen_t
            x_err     = self._person_x_err
            dist      = self._person_dist
            safety    = self._safety

        if not fm:
            return  # safety mode handles its own output via _vel_cb

        out = Twist()
        now = time.time()

        if not seen and (now - last_seen) > self._lost_timeout:
            # Person lost — stop and publish status
            self._vel_pub.publish(out)
            fs = String()
            fs.data = 'Lost person — stopped'
            self._fol_pub.publish(fs)
            return

        if safety == 'danger':
            # Too close — back up slowly
            out.linear.x = -0.05
            self._vel_pub.publish(out)
            fs = String(); fs.data = 'TOO CLOSE — backing up'
            self._fol_pub.publish(fs)
            return

        if seen:
            # Turn to center person horizontally
            # x_err: -1 = far left, 0 = centered, +1 = far right
            out.angular.z = -x_err * self._turn_gain

            # Approach or hold distance
            gap = dist - self._follow_dist   # positive = too far, negative = too close
            if gap > 0.3:
                # Move forward — faster when farther
                out.linear.x = min(self._follow_speed, gap * 0.25)
            elif gap < -0.2:
                # Too close — stop forward motion (danger zone handles reverse)
                out.linear.x = 0.0
            else:
                # In the sweet spot — just maintain heading
                out.linear.x = 0.0

            self._vel_pub.publish(out)

            fs = String()
            fs.data = (f'Following  dist={dist:.1f}m  '
                       f'turn={"L" if out.angular.z > 0 else "R"}{abs(out.angular.z):.2f}  '
                       f'fwd={out.linear.x:.2f}m/s')
            self._fol_pub.publish(fs)

    # ── safety mode velocity filter ───────────────────────────────────────────

    def _vel_cb(self, msg: Twist):
        with self._lock:
            fm = self._follow_mode

        if fm:
            return  # follow-me mode generates its own velocities

        cam_age = time.time() - self._last_cam_t
        if self._model is None or cam_age > self._cam_timeout:
            self._vel_pub.publish(msg)
            return

        with self._lock:
            lvl = self._safety

        if lvl == 'danger':
            self._vel_pub.publish(Twist())
        elif lvl == 'caution':
            out = Twist()
            out.linear.x  = msg.linear.x  * self._spd_caution
            out.linear.y  = msg.linear.y  * self._spd_caution
            out.angular.z = msg.angular.z * self._spd_caution
            self._vel_pub.publish(out)
        else:
            self._vel_pub.publish(msg)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _find_model(self) -> str:
        candidates = [
            os.path.expanduser(
                '~/autonomus-wheelchair-grad-project-nav2'
                '/autonomous_wheelchair/yolov8n.pt'
            ),
            'yolov8n.pt',
        ]
        for c in candidates:
            c = os.path.abspath(c)
            if os.path.exists(c):
                return c
        return ''

    def _diag(self):
        with self._lock:
            lvl  = self._safety
            dist = self._nearest_m
            fm   = self._follow_mode
            seen = self._person_seen
        age = time.time() - self._last_cam_t
        cam = f'{age:.0f}s ago' if self._last_cam_t else 'no feed'
        mode = f'FOLLOW-ME (person {"visible" if seen else "LOST"})' if fm else 'SAFETY'
        self.get_logger().info(
            f'[{mode}]  safety={lvl}  nearest={dist:.1f}m  cam={cam}'
        )

    def destroy_node(self):
        if self._show_win:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
