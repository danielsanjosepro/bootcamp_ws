#!/usr/bin/env python3
"""
ROS 2 node that controls bag recording via topic commands.

This node subscribes to /recording_commands and accepts:
- "record": Start a new recording
- "save": Save the current recording
- "delete": Delete the current recording

Usage:
    python controlled_ros2_bag_recording.py --config config_data_recorder.yml --output /path/to/bags
"""

from rich import print

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path
import signal

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RecordingControlNode(Node):
    """ROS 2 node for controlling bag recording via commands."""

    def __init__(self, topics: list[str], output_dir: Path):
        super().__init__("recording_control_node")

        self.topics = topics
        self.output_dir = output_dir.resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.recording_process = None
        self.current_bag_path = None
        self.saved_episodes_count = 0

        # Subscribe to recording commands
        self.subscription = self.create_subscription(
            String, "/recording_commands", self.command_callback, 10
        )

        self.get_logger().info("Recording control node started")
        self.get_logger().info(f"Listening for commands on /recording_commands")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(
            "Commands: 'record' (start), 'save' (keep), 'delete' (discard)"
        )
        self.get_logger().info(f"Saved episodes: {self.saved_episodes_count}")

    def command_callback(self, msg: String):
        """Handle incoming recording commands."""
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: '{command}'")

        if command == "record":
            self.start_recording()
        elif command == "save":
            self.save_recording()
        elif command == "delete":
            self.delete_recording()
        else:
            self.get_logger().warn(
                f"Unknown command: '{command}'. Valid commands: record, save, delete"
            )

    def start_recording(self):
        """Start a new bag recording."""
        if self.recording_process is not None:
            self.get_logger().warn(
                "Recording already in progress. Stop current recording first."
            )
            return

        # Create timestamped directory for this recording
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_bag_path = self.output_dir / f"bag_{timestamp}"

        # Build ros2 bag record command
        cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", str(self.current_bag_path)]
        cmd.extend(self.topics)

        try:
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
            )
            self.get_logger().info(f"Started recording to: {self.current_bag_path}")
            self.get_logger().info(f"Recording {len(self.topics)} topics")
        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")
            self.recording_process = None
            self.current_bag_path = None

    def stop_recording(self):
        """Stop the current recording process."""
        if self.recording_process is None:
            return

        try:
            # Send SIGINT to gracefully stop recording
            self.recording_process.send_signal(signal.SIGINT)
            self.recording_process.wait(timeout=5)
            self.get_logger().info("Recording stopped")
        except subprocess.TimeoutExpired:
            self.get_logger().warn("Recording did not stop gracefully, terminating")
            self.recording_process.terminate()
            self.recording_process.wait(timeout=2)
        except Exception as e:
            self.get_logger().error(f"Error stopping recording: {e}")

        self.recording_process = None

    def save_recording(self):
        """Save the current recording."""
        if self.recording_process is None:
            self.get_logger().warn("No recording in progress to save")
            return

        self.stop_recording()
        self.saved_episodes_count += 1
        self.get_logger().info(f"Recording saved at: {self.current_bag_path}")
        self.get_logger().info(
            f"Total saved episodes: {self.saved_episodes_count}"
        )
        self.current_bag_path = None

    def delete_recording(self):
        """Delete the current recording."""
        if self.recording_process is None:
            self.get_logger().warn("No recording in progress to delete")
            return

        self.stop_recording()

        # Delete the recording directory
        if self.current_bag_path and self.current_bag_path.exists():
            try:
                import shutil

                shutil.rmtree(self.current_bag_path)
                self.get_logger().info(f"Recording deleted: {self.current_bag_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to delete recording: {e}")

        self.current_bag_path = None

    def cleanup(self):
        """Clean up resources on shutdown."""
        if self.recording_process is not None:
            self.get_logger().info("Cleaning up active recording...")
            self.stop_recording()


def load_config(config_path: Path) -> list[str]:
    """Load ROS topics from YAML configuration file.

    Args:
        config_path: Path to the YAML configuration file

    Returns:
        List of ROS topic names

    Raises:
        FileNotFoundError: If config file doesn't exist
        ValueError: If config format is invalid
    """
    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_path) as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict) or "ros_topics" not in config:
        raise ValueError("Config file must contain 'ros_topics' key")

    topics = config["ros_topics"]
    if not isinstance(topics, list):
        raise ValueError("'ros_topics' must be a list")

    if not topics:
        raise ValueError("No topics specified in configuration")

    return topics


def main() -> None:
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description="ROS 2 node for controlled bag recording via commands",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "-c",
        "--config",
        type=Path,
        required=True,
        help="Path to YAML configuration file",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        required=True,
        help="Output directory for bag files",
    )

    args = parser.parse_args()

    try:
        topics = load_config(args.config)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    # Initialize ROS 2
    rclpy.init()

    node = RecordingControlNode(topics, args.output)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down recording control node...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
