#!/usr/bin/env python3
"""
ESP32 Bridge Node — ROS2 hardware interface for the autonomous wheelchair.

Reads:  JSON sensor stream from ESP32 (all-zero until sensors are wired up)
Writes: Differential drive M<left>,<right> commands from /cmd_vel

Publishes:
  /odom        — wheel odometry (currently all-zero; firmware has no sensor reads yet)
  /imu/data    — MPU6050 IMU (currently all-zero; firmware has no sensor reads yet)

NOTE: odom→base_link TF is published as a static identity transform in navigation.launch.py.
      Localization is handled entirely by slam_toolbox (map→odom TF from scan matching).

Subscribes:
  /cmd_vel     — Twist velocity commands (linear.x, angular.z) → M<l>,<r> to ESP32

Serial protocol to ESP32:
  M<left_pwm>,<right_pwm>\n  — autonomous differential drive
  W/S/A/D/X/B               — manual mode (from keyboard)
"""

import glob
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial


# ── helpers ──────────────────────────────────────────────────────────────────

def find_esp32_port():
    # ESP32 is always plugged in first → lowest ttyUSB index
    ports = sorted(glob.glob('/dev/ttyUSB*'))
    if ports:
        print(f'[esp32_bridge] Auto-detected ESP32 port: {ports[0]}')
        return ports[0]
    acm = sorted(glob.glob('/dev/ttyACM*'))
    if acm:
        return acm[0]
    print('[esp32_bridge] WARNING: no serial port found, using /dev/ttyUSB0')
    return '/dev/ttyUSB0'


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


# ── bridge node ───────────────────────────────────────────────────────────────

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('serial_port',      find_esp32_port())
        self.declare_parameter('baud_rate',        115200)
        self.declare_parameter('wheel_radius',     0.27)   # m — real back wheel
        self.declare_parameter('wheel_separation', 0.52)   # m — left-to-right wheel centre
        self.declare_parameter('encoder_ppr',      20)     # pulses per revolution

        # Velocity → PWM mapping.
        # max_linear_vel: the m/s the chair reaches at max_pwm.
        # Tune by measuring actual speed at a known PWM.
        self.declare_parameter('max_pwm',        200)    # PWM limit sent to ESP32
        self.declare_parameter('max_linear_vel', 0.5)    # m/s at max_pwm

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        # ── serial ──────────────────────────────────────────────────────────
        self.get_logger().info(f'Opening serial: {port} @ {baud}')
        self._ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)           # wait for ESP32 boot
        self._ser_lock = threading.Lock()

        # ── publishers ──────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, '/odom',     10)
        self._imu_pub  = self.create_publisher(Imu,      '/imu/data', 10)

        # ── subscriber ──────────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ── odometry state ───────────────────────────────────────────────────
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._last_left  = None
        self._last_right = None
        self._last_t     = None


        # ── reader thread ────────────────────────────────────────────────────
        self._alive = True
        threading.Thread(target=self._reader, daemon=True, name='esp32-rx').start()

        self.get_logger().info('ESP32 bridge ready. Waiting for sensor JSON ...')

    # ── cmd_vel → differential drive ─────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z

        r   = self.get_parameter('wheel_radius').value
        sep = self.get_parameter('wheel_separation').value
        max_pwm = self.get_parameter('max_pwm').value
        max_vel = self.get_parameter('max_linear_vel').value

        # Dead zone — send M0,0 (not 'X') so firmware ramp-down is not triggered
        if abs(lin) < 0.01 and abs(ang) < 0.01:
            with self._ser_lock:
                self._ser.write(b'M0,0\n')
            return

        # Differential drive kinematics: split Twist into left/right wheel velocities
        v_left  = lin - ang * sep / 2.0
        v_right = lin + ang * sep / 2.0

        # Scale to PWM range; clamp to max_pwm
        scale   = max_pwm / max_vel
        pwm_l   = int(max(-max_pwm, min(max_pwm, v_left  * scale)))
        pwm_r   = int(max(-max_pwm, min(max_pwm, v_right * scale)))

        cmd = f'M{pwm_l},{pwm_r}\n'.encode()
        with self._ser_lock:
            self._ser.write(cmd)

    # ── serial reader thread ─────────────────────────────────────────────────

    def _reader(self):
        while self._alive:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='replace').strip()
                if not line.startswith('{'):
                    # Print startup messages and diagnostics from ESP32
                    if line:
                        self.get_logger().info(f'[ESP32] {line}')
                    continue
                data = json.loads(line)
                self._process(data)
            except json.JSONDecodeError:
                pass
            except Exception as e:
                if self._alive:
                    self.get_logger().warn(f'Serial read error: {e}')
                    time.sleep(0.05)

    # ── process one sensor JSON frame ────────────────────────────────────────

    def _process(self, data: dict):
        now   = self.get_clock().now()
        stamp = now.to_msg()

        r    = self.get_parameter('wheel_radius').value
        sep  = self.get_parameter('wheel_separation').value
        ppr  = self.get_parameter('encoder_ppr').value
        m_per_tick = (2.0 * math.pi * r) / ppr

        lx = int(data['lx'])
        rx = int(data['rx'])

        # ── odometry ─────────────────────────────────────────────────────────
        vx = vy = vth = 0.0
        if self._last_left is not None:
            dt = (now.nanoseconds - self._last_t.nanoseconds) * 1e-9
            if 0 < dt < 0.5:
                dl = (lx - self._last_left)  * m_per_tick
                dr = (rx - self._last_right) * m_per_tick

                dist   = (dl + dr) / 2.0
                dtheta = (dr - dl) / sep

                self._theta += dtheta
                self._x     += dist * math.cos(self._theta)
                self._y     += dist * math.sin(self._theta)

                vx  = dist   / dt
                vth = dtheta / dt

        self._last_left  = lx
        self._last_right = rx
        self._last_t     = now

        q = euler_to_quaternion(0.0, 0.0, self._theta)

        # publish /odom
        odom = Odometry()
        odom.header.stamp          = stamp
        odom.header.frame_id       = 'odom'
        odom.child_frame_id        = 'base_link'
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vth
        odom.pose.covariance[0]    = 0.01   # x
        odom.pose.covariance[7]    = 0.01   # y
        odom.pose.covariance[35]   = 0.05   # yaw
        odom.twist.covariance[0]   = 0.01
        odom.twist.covariance[35]  = 0.05
        self._odom_pub.publish(odom)

        # odom → base_link TF is published as a static transform in navigation.launch.py
        # (no real wheel encoders, so dynamic odom is meaningless)

        # publish /imu/data  (raw — no orientation, MPU6050 has no magnetometer)
        imu = Imu()
        imu.header.stamp    = stamp
        imu.header.frame_id = 'imu_link'
        imu.linear_acceleration.x = float(data['ax'])
        imu.linear_acceleration.y = float(data['ay'])
        imu.linear_acceleration.z = float(data['az'])
        imu.angular_velocity.x    = float(data['gx'])
        imu.angular_velocity.y    = float(data['gy'])
        imu.angular_velocity.z    = float(data['gz'])
        imu.orientation_covariance[0]         = -1.0  # no orientation
        imu.linear_acceleration_covariance[0] =  0.04
        imu.linear_acceleration_covariance[4] =  0.04
        imu.linear_acceleration_covariance[8] =  0.04
        imu.angular_velocity_covariance[0]    =  0.02
        imu.angular_velocity_covariance[4]    =  0.02
        imu.angular_velocity_covariance[8]    =  0.02
        self._imu_pub.publish(imu)

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._alive = False
        try:
            with self._ser_lock:
                self._ser.write(b'X')   # stop motors on shutdown
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
