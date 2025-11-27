#!/usr/bin/env python3
"""
Script to start ROS 2 bag recording from a YAML configuration file.

Usage:
    python start_ros2_bag_recording.py --config config_data_recorder.yml
    python start_ros2_bag_recording.py --config config_data_recorder.yml --output /path/to/bags
"""

from rich import print

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path

import yaml


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


def start_recording(topics: list[str], output_dir: Path | None = None) -> None:
    """Start ROS 2 bag recording with specified topics.

    Args:
        topics: List of ROS topic names to record
        output_dir: Optional directory to save bag files
    """
    cmd = ["ros2", "bag", "record"]

    # Use MCAP storage format
    cmd.extend(["-s", "mcap"])

    if output_dir:
        # Convert to absolute path to ensure correct location
        output_dir = output_dir.resolve()

        # Create timestamped subdirectory to avoid conflicts
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = output_dir / f"bag_{timestamp}"

        output_dir.mkdir(parents=True, exist_ok=True)
        cmd.extend(["-o", str(bag_dir)])
        print(f"Output directory: {bag_dir}")

    cmd.extend(topics)

    print(f"Starting ROS 2 bag recording with {len(topics)} topics...")
    print(f"Command: {' '.join(cmd)}")
    print("\nPress Ctrl+C to stop recording\n")

    try:
        subprocess.run(cmd, check=True)
    except KeyboardInterrupt:
        print("\n\nRecording stopped by user")
    except subprocess.CalledProcessError as e:
        print(
            f"\nError: Recording failed with exit code {e.returncode}", file=sys.stderr
        )
        sys.exit(1)
    except FileNotFoundError:
        print("\nError: ros2 command not found. Is ROS 2 sourced?", file=sys.stderr)
        sys.exit(1)


def main() -> None:
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(
        description="Start ROS 2 bag recording from YAML configuration",
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
        help="Output directory for bag files (optional)",
    )

    args = parser.parse_args()

    try:
        topics = load_config(args.config)
        start_recording(topics, args.output)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
