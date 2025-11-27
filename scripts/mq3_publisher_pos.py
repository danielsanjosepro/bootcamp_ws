#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from simpub.core.node_manager import init_xr_node_manager
from simpub.xr_device.meta_quest3 import MetaQuest3
from std_msgs.msg import Float32
from tf2_ros import (
    TransformBroadcaster,
    TransformListener,
    Buffer,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
from scipy.spatial.transform import Rotation as R


class Pose:
    """A pose representation with translation and rotation."""

    def __init__(self, translation, rotation):
        """
        Initialize a Pose.

        Args:
            translation: numpy array [x, y, z]
            rotation: scipy.spatial.transform.Rotation object
        """
        self.translation = np.array(translation)
        self.rotation = rotation

    @classmethod
    def from_pos_quat(cls, pos, quat):
        """
        Create a Pose from position and quaternion.

        Args:
            pos: list or array [x, y, z]
            quat: list or array [x, y, z, w] (quaternion)
        """
        translation = np.array(pos)
        rotation = R.from_quat(quat)
        return cls(translation, rotation)

    @classmethod
    def from_transform_msg(cls, transform):
        """
        Create a Pose from a ROS TransformStamped message.

        Args:
            transform: geometry_msgs.msg.TransformStamped
        """
        pos = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
        quat = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        return cls.from_pos_quat(pos, quat)

    def inverse(self):
        """Compute the inverse of this pose."""
        inv_rot = self.rotation.inv()
        inv_trans = -inv_rot.apply(self.translation)
        return Pose(inv_trans, inv_rot)

    def __sub__(self, other):
        """
        Compute the relative transform from other to self.
        Returns the delta pose: self = other + delta

        Args:
            other: Another Pose object

        Returns:
            Pose: The relative transformation
        """
        # Compute relative rotation
        delta_rot = other.rotation.inv() * self.rotation

        # Compute relative translation (in the frame of 'other')
        delta_trans = other.rotation.inv().apply(self.translation - other.translation)

        return Pose(delta_trans, delta_rot)

    def __add__(self, delta):
        """
        Apply a relative transform (delta) to this pose.

        Args:
            delta: A Pose object representing relative transformation

        Returns:
            Pose: The resulting pose after applying the delta
        """
        # Apply rotation and translation
        new_trans = self.translation + self.rotation.apply(delta.translation)
        new_rot = self.rotation * delta.rotation

        return Pose(new_trans, new_rot)

    def to_dict(self):
        """Convert pose to dictionary format."""
        return {
            "translation": self.translation.tolist(),
            "rotation_quat": self.rotation.as_quat().tolist(),
            "rotation_euler": self.rotation.as_euler("xyz").tolist(),
        }


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
        self._get_latest_rate = 20  # Hz

        # TF broadcaster for transforms
        self._is_teleop_from_behind = True
        self._left_arm_side = "left" if not self._is_teleop_from_behind else "right"
        self._right_arm_side = "right" if not self._is_teleop_from_behind else "left"

        self.tf_broadcaster = TransformBroadcaster(self)

        # TF listener for getting arm poses
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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

        self.create_timer(1.0 / self._get_latest_rate, self._callback_get_latest_tf)
        self.create_timer(
            1.0 / self._retrieval_rate, self._callback_get_controller_data
        )
        self.create_timer(1.0 / self._publish_rate, self._publish_twist)
        self._latest_pose_left = None
        self._latest_pose_right = None

        # Reference poses for pose-based control
        self._controller_ref_pose_left = None
        self._controller_ref_pose_right = None
        self._robot_ref_pose_left = None
        self._robot_ref_pose_right = None
        self._teleop_active_left = False
        self._teleop_active_right = False

        # PD controller gains
        self._kp_linear = 1.5  # Proportional gain for linear motion
        self._kd_linear = 0.1  # Derivative gain for linear motion
        self._kp_angular = 5.0  # Proportional gain for angular motion
        self._kd_angular = 0.05  # Derivative gain for angular motion

        # Velocity limits (safety)
        self._max_linear_vel = 0.5  # m/s - maximum linear velocity
        self._max_angular_vel = 1.0  # rad/s - maximum angular velocity

        # Control flags
        self._enable_orientation_control = False  # Set to True to enable orientation control

        # Store previous errors and time for derivative term (per-arm)
        self._prev_error_left = None
        self._prev_error_right = None
        self._prev_time_left = self.get_clock().now()
        self._prev_time_right = self.get_clock().now()

        # Initialize controller input data
        self._latest_input_data = None
        input_data = self.mq3.get_controller_data()
        if input_data is not None:
            self._latest_input_data = input_data

    def _publish_twist(self):
        if self._latest_input_data is not None:
            self.send_twist(self._latest_input_data)

    def _callback_get_controller_data(self):
        """Callback to retrieve latest controller data."""
        input_data = self.mq3.get_controller_data()
        if input_data is not None:
            self._latest_input_data = input_data

    def _saturate_velocity(self, velocity_vec, max_vel):
        """
        Saturate a velocity vector to a maximum magnitude.

        Args:
            velocity_vec: numpy array of velocity
            max_vel: maximum allowed magnitude

        Returns:
            numpy array: saturated velocity
        """
        magnitude = np.linalg.norm(velocity_vec)
        if magnitude > max_vel:
            return velocity_vec * (max_vel / magnitude)
        return velocity_vec

    def compute_twist_from_pose_error(
        self, target_pose, current_pose, prev_error, prev_time, side="right"
    ):
        """
        Compute twist command using PD controller based on pose error.

        Args:
            target_pose: Pose object representing desired pose
            current_pose: Pose object representing current robot pose
            prev_error: Previous error for derivative term (Pose object or None)
            prev_time: Previous time for computing dt
            side: "left" or "right"

        Returns:
            tuple: (TwistStamped, error_pose, current_time)
        """
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = f"{side}_link0"

        # Compute pose error in base frame (link0)
        # Translational error: simple difference in base frame
        error_trans = target_pose.translation - current_pose.translation

        # Rotational error: relative rotation needed
        error_rot = current_pose.rotation.inv() * target_pose.rotation

        # Create error pose for tracking
        error_pose = Pose(error_trans, error_rot)

        # Compute time delta
        current_time = self.get_clock().now()
        dt = (current_time - prev_time).nanoseconds / 1e9

        if dt <= 0 or dt > 1.0:  # Guard against invalid dt
            dt = 1.0 / self._publish_rate

        # Compute derivative term
        if prev_error is not None:
            error_derivative_trans = (error_trans - prev_error.translation) / dt
            # For rotation, compute angular velocity difference
            error_rot_diff = prev_error.rotation.inv() * error_rot
            error_derivative_rot = error_rot_diff.as_rotvec() / dt
        else:
            error_derivative_trans = np.zeros(3)
            error_derivative_rot = np.zeros(3)

        # Apply PD control law (all in base frame)
        linear_command = self._kp_linear * error_trans + self._kd_linear * error_derivative_trans

        if self._enable_orientation_control:
            angular_command = self._kp_angular * error_rot.as_rotvec() + self._kd_angular * error_derivative_rot
        else:
            angular_command = np.zeros(3)  # Disable orientation control

        # Saturate velocities for safety
        linear_command = self._saturate_velocity(linear_command, self._max_linear_vel)
        angular_command = self._saturate_velocity(angular_command, self._max_angular_vel)

        # Populate twist message
        twist.twist.linear.x = linear_command[0]
        twist.twist.linear.y = linear_command[1]
        twist.twist.linear.z = linear_command[2]
        twist.twist.angular.x = angular_command[0]
        twist.twist.angular.y = angular_command[1]
        twist.twist.angular.z = angular_command[2]

        return twist, error_pose, current_time

    def convert_twist(self, input_data, side="right", linear_alpha=1.5, rot_alpha=10.0):
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
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        return twist

    def send_twist(self, input_data):
        """Send twist commands based on pose-based control."""
        self.get_logger().info("Publishing Twist Start", throttle_duration_sec=1.0)

        # Process left arm
        self._process_arm_control(input_data, self._left_arm_side)

        # Process right arm
        self._process_arm_control(input_data, self._right_arm_side)

        self.get_logger().info("Publishing Twist End", throttle_duration_sec=1.0)

    def _process_arm_control(self, input_data, side):
        """
        Process control for a single arm (left or right).

        Args:
            input_data: Controller data from Meta Quest 3
            side: "left" or "right"
        """
        # Get hand trigger value
        hand_trigger = input_data[side]["hand_trigger"]

        # Determine publishers based on side
        if side == self._left_arm_side:
            publisher = self.twist_publisher_left_
            gripper_pub = self.gripper_left_pub
        else:
            publisher = self.twist_publisher_right_
            gripper_pub = self.gripper_right_pub

        # Gripper control (index trigger)
        gripper_msg = Float32()
        gripper_msg.data = 1.0 if input_data[side]["index_trigger"] < 0.3 else -1.0
        gripper_pub.publish(gripper_msg)

        # Check if hand trigger is pressed (motion enabler)
        if hand_trigger > 0.7:
            # Capture reference poses when first pressed
            # Check instance variable directly to avoid local variable bug
            is_active = self._teleop_active_left if side == self._left_arm_side else self._teleop_active_right

            if not is_active:
                # Get controller pose from input data
                controller_pos = input_data[side]["pos"]
                controller_quat = input_data[side]["rot"]
                controller_ref_pose = Pose.from_pos_quat(
                    controller_pos, controller_quat
                )

                # Get robot pose from TF
                latest_robot_pose = self._latest_pose_left if side == self._left_arm_side else self._latest_pose_right

                if latest_robot_pose is not None:
                    robot_ref_pose = Pose.from_transform_msg(latest_robot_pose)

                    # Apply coordinate transformation for controller pose
                    controller_ref_pose = self._transform_controller_pose(
                        controller_ref_pose, side
                    )

                    # Store references
                    if side == self._left_arm_side:
                        self._controller_ref_pose_left = controller_ref_pose
                        self._robot_ref_pose_left = robot_ref_pose
                        self._teleop_active_left = True
                    else:
                        self._controller_ref_pose_right = controller_ref_pose
                        self._robot_ref_pose_right = robot_ref_pose
                        self._teleop_active_right = True

                    self.get_logger().info(
                        f"Reference poses captured for {side} arm",
                        throttle_duration_sec=1.0,
                    )
                else:
                    self.get_logger().warn(
                        f"Cannot capture reference: robot pose not available for {side}",
                        throttle_duration_sec=1.0,
                    )
                    return

            # Compute and send twist command
            # Read instance variables directly
            is_active = self._teleop_active_left if side == self._left_arm_side else self._teleop_active_right
            controller_ref_pose = self._controller_ref_pose_left if side == self._left_arm_side else self._controller_ref_pose_right
            robot_ref_pose = self._robot_ref_pose_left if side == self._left_arm_side else self._robot_ref_pose_right

            if is_active and controller_ref_pose is not None and robot_ref_pose is not None:
                # Get current controller pose
                controller_pos = input_data[side]["pos"]
                controller_quat = input_data[side]["rot"]
                current_controller_pose = Pose.from_pos_quat(
                    controller_pos, controller_quat
                )

                # Apply coordinate transformation
                current_controller_pose = self._transform_controller_pose(
                    current_controller_pose, side
                )

                # Compute relative transform from reference
                delta_controller = current_controller_pose - controller_ref_pose

                # Compute target robot pose
                target_robot_pose = robot_ref_pose + delta_controller

                # Get current robot pose
                latest_robot_pose = self._latest_pose_left if side == self._left_arm_side else self._latest_pose_right
                prev_error = self._prev_error_left if side == self._left_arm_side else self._prev_error_right
                prev_time = self._prev_time_left if side == self._left_arm_side else self._prev_time_right

                if latest_robot_pose is not None:
                    current_robot_pose = Pose.from_transform_msg(latest_robot_pose)

                    # Compute twist using PD controller
                    twist, error_pose, current_time = self.compute_twist_from_pose_error(
                        target_robot_pose, current_robot_pose, prev_error, prev_time, side
                    )

                    # Update previous error and time
                    if side == self._left_arm_side:
                        self._prev_error_left = error_pose
                        self._prev_time_left = current_time
                    else:
                        self._prev_error_right = error_pose
                        self._prev_time_right = current_time

                    # Publish twist
                    publisher.publish(twist)

                    self.get_logger().info(
                        f"{side} arm - Error: pos={np.linalg.norm(error_pose.translation):.3f}, "
                        f"rot={np.linalg.norm(error_pose.rotation.as_rotvec()):.3f}",
                        throttle_duration_sec=0.5,
                    )
        else:
            # Hand trigger released - reset teleop state and send zero twist
            is_active = self._teleop_active_left if side == self._left_arm_side else self._teleop_active_right

            if is_active:
                if side == self._left_arm_side:
                    self._teleop_active_left = False
                    self._controller_ref_pose_left = None
                    self._robot_ref_pose_left = None
                    self._prev_error_left = None
                else:
                    self._teleop_active_right = False
                    self._controller_ref_pose_right = None
                    self._robot_ref_pose_right = None
                    self._prev_error_right = None

                self.get_logger().info(
                    f"Teleop deactivated for {side} arm", throttle_duration_sec=1.0
                )

            # Send zero twist
            zero_twist = TwistStamped()
            zero_twist.header.stamp = self.get_clock().now().to_msg()
            zero_twist.header.frame_id = f"{side}_link0"
            zero_twist.twist.linear.x = 0.0
            zero_twist.twist.linear.y = 0.0
            zero_twist.twist.linear.z = 0.0
            zero_twist.twist.angular.x = 0.0
            zero_twist.twist.angular.y = 0.0
            zero_twist.twist.angular.z = 0.0
            publisher.publish(zero_twist)

    def _transform_controller_pose(self, controller_pose, side):
        """
        Transform controller pose using the coordinate transformation matrix.

        Args:
            controller_pose: Pose object from controller
            side: "left" or "right"

        Returns:
            Pose: Transformed pose
        """
        # Use the same rotation matrices as in the original convert_twist method
        if side == "left":
            rotation_matrix = np.array(
                [
                    [0.88057351, 0.40157173, 0.25165539],
                    [-0.44095002, 0.4996947, 0.74556575],
                    [0.17364727, -0.7674929, 0.61709097],
                ]
            )
        elif side == "right":
            rotation_matrix = np.array(
                [
                    [0.88057351, -0.40157173, 0.25165539],
                    [0.44095002, 0.4996947, -0.74556575],
                    [0.17364727, 0.7674929, 0.61709097],
                ]
            )
        else:
            raise ValueError("The side is not valid, either left or right.")

        # Transform translation
        transformed_trans = np.dot(rotation_matrix.T, controller_pose.translation)

        # Transform rotation
        # Convert rotation matrix to scipy Rotation
        coord_transform_rot = R.from_matrix(rotation_matrix.T)
        transformed_rot = coord_transform_rot * controller_pose.rotation

        return Pose(transformed_trans, transformed_rot)

    def _callback_get_latest_tf(self):
        """Callback to get the latest TF transforms for left and right arms."""
        try:
            # Get current time
            now = rclpy.time.Time()

            # Lookup transform for left arm (from base to end effector)
            # Using link0 as base and link8 (or ee_link) as end effector
            # Adjust frame names based on your robot's TF tree
            transform_left = self.tf_buffer.lookup_transform(
                f"{self._left_arm_side}_link0",  # target frame
                f"{self._left_arm_side}_link8",  # source frame (end effector)
                now,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            self._latest_pose_left = transform_left

            self.get_logger().info(
                f"Left arm pose: Position({transform_left.transform.translation.x:.3f}, "
                f"{transform_left.transform.translation.y:.3f}, "
                f"{transform_left.transform.translation.z:.3f}), "
                f"Orientation({transform_left.transform.rotation.x:.3f}, "
                f"{transform_left.transform.rotation.y:.3f}, "
                f"{transform_left.transform.rotation.z:.3f}, "
                f"{transform_left.transform.rotation.w:.3f})",
                throttle_duration_sec=1.0,
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(
                f"Could not get left arm transform: {e}", throttle_duration_sec=5.0
            )

        try:
            # Lookup transform for right arm
            now = rclpy.time.Time()
            transform_right = self.tf_buffer.lookup_transform(
                f"{self._right_arm_side}_link0",  # target frame
                f"{self._right_arm_side}_link8",  # source frame (end effector)
                now,
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
            self._latest_pose_right = transform_right

            self.get_logger().info(
                f"Right arm pose: Position({transform_right.transform.translation.x:.3f}, "
                f"{transform_right.transform.translation.y:.3f}, "
                f"{transform_right.transform.translation.z:.3f}), "
                f"Orientation({transform_right.transform.rotation.x:.3f}, "
                f"{transform_right.transform.rotation.y:.3f}, "
                f"{transform_right.transform.rotation.z:.3f}, "
                f"{transform_right.transform.rotation.w:.3f})",
                throttle_duration_sec=1.0,
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(
                f"Could not get right arm transform: {e}", throttle_duration_sec=5.0
            )


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
