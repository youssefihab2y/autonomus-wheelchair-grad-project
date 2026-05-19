#!/usr/bin/env python3
"""
Goal Relay — bridges /goal_pose topic → Nav2 NavigateToPose action.

The web dashboard publishes PoseStamped to /goal_pose via roslib.
Nav2's bt_navigator only accepts goals through its action server.
This node translates the topic into an action call so the website
can drive the wheelchair autonomously.

Subscribes: /goal_pose  (geometry_msgs/PoseStamped)
Action:     /navigate_to_pose  (nav2_msgs/NavigateToPose)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalRelay(Node):
    def __init__(self):
        super().__init__('goal_relay')

        self._client: ActionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._handle: ClientGoalHandle | None = None
        self._pending_cancel = False

        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.get_logger().info('Goal relay ready — waiting for /goal_pose')

    # ── receive goal from website ─────────────────────────────────────────────

    def _goal_cb(self, msg: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn(
                'navigate_to_pose action server not available — is Nav2 running?'
            )
            return

        # Cancel active goal before sending a new one
        if self._handle is not None:
            self.get_logger().info('New goal received — cancelling current navigation')
            self._handle.cancel_goal_async()
            self._handle = None

        # Ensure frame_id is set
        if not msg.header.frame_id:
            msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = msg

        self.get_logger().info(
            f'Sending goal → x={msg.pose.position.x:.2f}  y={msg.pose.position.y:.2f}'
        )

        self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        ).add_done_callback(self._response_cb)

    # ── action callbacks ──────────────────────────────────────────────────────

    def _response_cb(self, future):
        handle: ClientGoalHandle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            return
        self._handle = handle
        self.get_logger().info('Goal accepted — navigating…')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = getattr(fb, 'distance_remaining', None)
        if dist is not None:
            self.get_logger().debug(f'Distance remaining: {dist:.2f} m')

    def _result_cb(self, future):
        self._handle = None
        result = future.result()
        status = result.status
        # status 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        labels = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        label  = labels.get(status, f'status={status}')
        if status == 4:
            self.get_logger().info(f'Navigation {label} ✓')
        else:
            self.get_logger().warn(f'Navigation {label}')


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = GoalRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
