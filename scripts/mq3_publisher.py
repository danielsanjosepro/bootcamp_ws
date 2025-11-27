#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from simpub.core.node_manager import init_xr_node_manager
from simpub.xr_device.meta_quest3 import MetaQuest3
from std_msgs.msg import Float32, String
from tf2_ros import TransformBroadcaster


class MetaQuest3Publisher(Node):
    def __init__(self):
        super().__init__("metaquest3_publisher")

        # Initialize XR node manager
        self.get_logger().info("Initializing XR node manager...")
        self.net_manager = init_xr_node_manager("192.168.0.117")
        self.net_manager.start_discover_node_loop()

        # Initialize Meta Quest 3
        self.mq3 = MetaQuest3("BootCamp-1")
        self.get_logger().info("MetaQuest3 initialized.")
        # ROS 2 publisher for structured controller data

        self._publish_rate = 30  # Hz
        self._retrieval_rate = 60  # Hz

        # TF broadcaster for transforms
        self._is_teleop_from_behind = True
        self._left_arm_side = "left" if not self._is_teleop_from_behind else "right"
        self._right_arm_side = "right" if not self._is_teleop_from_behind else "left"

        self.tf_broadcaster = TransformBroadcaster(self)
        self.twist_publisher_left_ = self.create_publisher(
            TwistStamped,
            f"/franka_robot/{self._left_arm_side}/f_30hz/teleop/twist_stamped",
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.twist_publisher_right_ = self.create_publisher(
            TwistStamped,
            f"/franka_robot/{self._right_arm_side}/f_30hz/teleop/twist_stamped",
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.gripper_left_pub = self.create_publisher(
            Float32,
            f"/robotiq_gripper/{self._left_arm_side}/f_30hz/robotiq_2f_gripper/confidence_command",
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.gripper_right_pub = self.create_publisher(
            Float32,
            f"/robotiq_gripper/{self._right_arm_side}/f_30hz/robotiq_2f_gripper/confidence_command",
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self._latest_input_data = None
        self.create_timer(
            1.0 / self._publish_rate,
            self._publish_twist,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(
            1.0 / self._retrieval_rate,
            self._retrieve_input_data,
            callback_group=ReentrantCallbackGroup(),
        )
        self._record_data_publisher = self.create_publisher(
            String,
            "/recording_commands",
            10,
            callback_group=ReentrantCallbackGroup(),
        )
        self.create_timer(1.0 / 10.0, self._send_record_command)

    def _retrieve_input_data(self):
        input_data = self.mq3.get_controller_data()
        if input_data is not None:
            self._latest_input_data = input_data

    def _publish_twist(self):
        if self._latest_input_data is not None:
            self.send_twist(self._latest_input_data)

    def convert_twist(self, input_data, side="right", linear_alpha=1.5, rot_alpha=2.0):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = f"{side}_link0"

        if side == "left":
            rotation = np.array(
                [
                    [0.88057351, 0.40157173, 0.25165539],
                    [-0.44095002, 0.4996947, 0.74556575],
                    [0.17364727, -0.7674929, 0.61709097],
                ]
            )
        elif side == "right":
            rotation = np.array(
                [
                    [0.88057351, -0.40157173, 0.25165539],
                    [0.44095002, 0.4996947, -0.74556575],
                    [0.17364727, 0.7674929, 0.61709097],
                ]
            )
        else:
            raise ValueError("The side is not valid, either left or right.")

        delta_pos = np.array(
            [
                input_data[side]["vel"][0],
                input_data[side]["vel"][1],
                input_data[side]["vel"][2],
            ]
        )
        delta_rot = np.array(
            [
                input_data[side]["ang_vel"][0],
                input_data[side]["ang_vel"][1],
                input_data[side]["ang_vel"][2],
            ]
        )

        delta_pos_transformed = linear_alpha * np.dot(rotation.T, delta_pos)
        delta_rot_transformed = rot_alpha * np.dot(rotation.T, delta_rot)

        twist.twist.linear.x = delta_pos_transformed[0]
        twist.twist.linear.y = delta_pos_transformed[1]
        twist.twist.linear.z = delta_pos_transformed[2]
        twist.twist.angular.x = delta_rot_transformed[0]
        twist.twist.angular.y = delta_rot_transformed[1]
        twist.twist.angular.z = delta_rot_transformed[2]
        return twist

    def send_twist(self, input_data):
        self.get_logger().info("Publishing Twist Start", throttle_duration_sec=1.0)

        left_twist = self.convert_twist(input_data, side=self._left_arm_side)

        Float32_msg_left = Float32()
        Float32_msg_left.data = (
            1.0 if input_data[self._left_arm_side]["index_trigger"] < 0.3 else -1.0
        )

        if input_data[self._left_arm_side]["hand_trigger"] > 0.7:
            self.twist_publisher_left_.publish(left_twist)
        else:
            left_twist.twist.linear.x = 0.0
            left_twist.twist.linear.y = 0.0
            left_twist.twist.linear.z = 0.0
            left_twist.twist.angular.x = 0.0
            left_twist.twist.angular.y = 0.0
            left_twist.twist.angular.z = 0.0
            self.twist_publisher_left_.publish(left_twist)

        self.gripper_left_pub.publish(Float32_msg_left)
        right_twist = self.convert_twist(input_data, side=self._right_arm_side)

        Float32_msg_right = Float32()
        Float32_msg_right.data = (
            1.0 if input_data[self._right_arm_side]["index_trigger"] < 0.3 else -1.0
        )
        if input_data[self._right_arm_side]["hand_trigger"] > 0.7:
            self.twist_publisher_right_.publish(right_twist)

        else:
            right_twist.twist.linear.x = 0.0
            right_twist.twist.linear.y = 0.0
            right_twist.twist.linear.z = 0.0
            right_twist.twist.angular.x = 0.0
            right_twist.twist.angular.y = 0.0
            right_twist.twist.angular.z = 0.0
            self.twist_publisher_right_.publish(right_twist)
        self.gripper_right_pub.publish(Float32_msg_right)
        self.get_logger().info("Publishing Twist End", throttle_duration_sec=1.0)

    def _send_record_command(self):
        if self._latest_input_data is None:
            self.get_logger().warn(
                "Can not send record data since input data is still None",
                throttle_duration_sec=1.0,
            )
            return

        msg = String()
        # self.get_logger().info("Checking buttons for recording command")
        self.get_logger().info(
            f"Button states - A: {self._latest_input_data['A']}, B: {self._latest_input_data['B']}, X: {self._latest_input_data['X']}",
            throttle_duration_sec=1.0,
        )
        if self._latest_input_data["A"]:
            self.get_logger().info("Button A pressed - sending 'save' command")
            msg.data = "save"
        elif self._latest_input_data["B"]:
            self.get_logger().info("Button B pressed - sending 'delete' command")
            msg.data = "delete"
        elif self._latest_input_data["X"]:
            self.get_logger().info("Button X pressed - sending 'record' command")
            msg.data = "record"
        else:
            return  # No button pressed; do not send any command

        self._record_data_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MetaQuest3Publisher()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("MetaQuest3 publisher stopped by user.")
    except Exception as e:
        node.get_logger().error(f"Error in publishing loop: {e}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
