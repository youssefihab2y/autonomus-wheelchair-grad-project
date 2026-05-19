#!/usr/bin/env python3
"""
IMU Attitude Test Node — complementary filter on raw MPU6050 data.

Subscribes : /imu/data  (sensor_msgs/Imu  — raw accel + gyro, no orientation)
Publishes  : /imu/attitude  (Imu with orientation filled in)
             /imu/rpy       (geometry_msgs/Vector3Stamped — roll/pitch/yaw in radians)

Roll & Pitch: complementary filter (0.98 gyro + 0.02 accel) — reliable, gravity-referenced.
Yaw        : pure gyro integration — drifts without magnetometer. Reset with Ctrl+C.

Assumes default MPU6050 chip orientation:
  X = forward,  Y = left,  Z = up  (chip flat with text readable, connector back).
If your chip is rotated 180° or mounted differently, axes will be wrong — adjust then.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3Stamped
from sensor_msgs.msg import Imu


ALPHA = 0.98          # complementary filter gyro weight
PRINT_HZ = 4          # terminal print rate (times/sec)
STALE_THRESHOLD = 0.5 # seconds — reset filter on stale data


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


class AttitudeNode(Node):
    def __init__(self):
        super().__init__('imu_attitude_test')

        self._roll  = 0.0
        self._pitch = 0.0
        self._yaw   = 0.0
        self._last_t = None
        self._msg_count = 0

        self._att_pub = self.create_publisher(Imu,           '/imu/attitude', 10)
        self._rpy_pub = self.create_publisher(Vector3Stamped, '/imu/rpy',     10)

        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_timer(1.0 / PRINT_HZ, self._print_cb)

        self.get_logger().info(
            'Attitude test ready.\n'
            '  Listening on /imu/data\n'
            '  Roll & Pitch: complementary filter (gravity-referenced)\n'
            '  Yaw         : gyro integration only — WILL DRIFT\n'
        )

    # ── IMU callback ───────────────────────────────────────────────────────────

    def _imu_cb(self, msg: Imu):
        now = self.get_clock().now()
        self._msg_count += 1

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Accel-only estimate (good reference when not accelerating hard)
        # Protect against divide-by-zero when az ~ 0
        roll_acc  = math.atan2(ay, math.sqrt(ax * ax + az * az))
        pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        # Time delta
        dt = 0.0
        if self._last_t is not None:
            dt = (now.nanoseconds - self._last_t.nanoseconds) * 1e-9
        self._last_t = now

        if dt > 0.0 and dt < STALE_THRESHOLD:
            self._roll  = ALPHA * (self._roll  + gx * dt) + (1.0 - ALPHA) * roll_acc
            self._pitch = ALPHA * (self._pitch + gy * dt) + (1.0 - ALPHA) * pitch_acc
            self._yaw  += gz * dt
        else:
            # First sample or stale gap: seed from accelerometer
            self._roll  = roll_acc
            self._pitch = pitch_acc

        stamp = msg.header.stamp

        # Publish /imu/rpy  (radians)
        rpy = Vector3Stamped()
        rpy.header.stamp    = stamp
        rpy.header.frame_id = 'imu_link'
        rpy.vector.x = self._roll
        rpy.vector.y = self._pitch
        rpy.vector.z = self._yaw
        self._rpy_pub.publish(rpy)

        # Publish /imu/attitude (Imu with orientation)
        imu_out = Imu()
        imu_out.header.stamp    = stamp
        imu_out.header.frame_id = 'imu_link'
        imu_out.linear_acceleration           = msg.linear_acceleration
        imu_out.angular_velocity              = msg.angular_velocity
        imu_out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        imu_out.angular_velocity_covariance   = msg.angular_velocity_covariance
        imu_out.orientation = euler_to_quaternion(self._roll, self._pitch, self._yaw)
        # Low confidence on yaw (drifts without magnetometer)
        imu_out.orientation_covariance[0] = 0.05   # roll  variance
        imu_out.orientation_covariance[4] = 0.05   # pitch variance
        imu_out.orientation_covariance[8] = 0.50   # yaw   variance (high)
        self._att_pub.publish(imu_out)

    # ── terminal display ───────────────────────────────────────────────────────

    def _print_cb(self):
        if self._msg_count == 0:
            print('\r  [waiting for /imu/data ...]                              ', end='', flush=True)
            return
        r = math.degrees(self._roll)
        p = math.degrees(self._pitch)
        y = math.degrees(self._yaw)
        print(
            f'\r  ROLL: {r:+7.2f}°   PITCH: {p:+7.2f}°   YAW: {y:+8.2f}° (drift)',
            end='', flush=True,
        )


# ── entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = AttitudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()   # newline after the rolling display
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
