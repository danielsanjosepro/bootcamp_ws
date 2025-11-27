# ROS 2 Bag Recording Guide

This workspace provides tools for recording ROS 2 bag files in MCAP format using SimPublisher and a BiManual manipulation robot.

## Install 

```
pixi shell -e humble
export ROS_DOMAIN_ID = 100
ros2 daemon stop

ros2 topic list  # check that you find the topics
```

## Start teleop

```
python scripts/mq3_publisher.py
```

Set the MetaQuest 3 on and be sure to be connected to the wifi. THen move the red axis in point outward of the robot.

## Recording Scripts

### 1. Direct Recording Script

**File:** `scripts/start_ros2_bag_recording.py`

Simple script that starts recording immediately and runs until you stop it.

**Usage:**
```bash
python scripts/start_ros2_bag_recording.py --config config/config_data_recorder.yml --output ./outputs/recordings
```

**Arguments:**
- `-c, --config`: Path to YAML config file with topic list (required)
- `-o, --output`: Output directory for bag files (optional)

**Stopping:**
- Press `Ctrl+C` to stop recording

**Output:**
- Bags saved in timestamped subdirectories: `bag_YYYYMMDD_HHMMSS/`
- Format: MCAP

---

### 2. Controlled Recording Node

**File:** `scripts/controlled_ros2_bag_recording.py`

ROS 2 node that controls recording via topic commands. Useful for episodic data collection where you want to decide whether to keep or discard recordings.

**Usage:**
```bash
python scripts/controlled_ros2_bag_recording.py --config config/config_data_recorder.yml --output ./outputs/episodes
```

**Arguments:**
- `-c, --config`: Path to YAML config file with topic list (required)
- `-o, --output`: Output directory for bag files (required)

**Commands:**

Send commands via the `/recording_commands` topic:

```bash
# Start a new recording episode
ros2 topic pub --once /recording_commands std_msgs/msg/String "{data: 'record'}"

# Save the current episode (keeps the recording)
ros2 topic pub --once /recording_commands std_msgs/msg/String "{data: 'save'}"

# Delete the current episode (discards the recording)
ros2 topic pub --once /recording_commands std_msgs/msg/String "{data: 'delete'}"
```

**Features:**
- Tracks number of saved episodes
- Prevents starting a new recording while one is active
- Automatically deletes unwanted recordings
- Timestamped directories for each episode

**Typical Workflow:**
1. Start the node
2. Send `record` command to start capturing
3. Perform your task/demonstration
4. Send `save` to keep the episode, or `delete` to discard it
5. Repeat steps 2-4 for multiple episodes

**Output:**
- Saved episodes in timestamped subdirectories: `bag_YYYYMMDD_HHMMSS/`
- Format: MCAP
- Episode counter logged on each save

---

## Configuration

The YAML config file specifies which topics to record:

**Example:** `config/config_data_recorder.yml`
```yaml
ros_topics:
  - '/parameter_events'
  - '/rosout'
  - '/tf'
  - '/tf_static'
  - '/left/franka_robot_state_broadcaster/measured_joint_states'
  - '/right/franka_robot_state_broadcaster/measured_joint_states'
  # ... add your topics here
```

## Quick Start: Recording Episodes

1. **Start the controlled recording node:**
   ```bash
   python scripts/controlled_ros2_bag_recording.py \
     --config config/config_data_recorder.yml \
     --output ./my_episodes
   ```
2. **In a different terminal start the teleoperation setup to publish twists to a bimanual setup**

3. **Record episodes by:*
    - Press X to start recording
    - Press A to save the episode
    - Press B to delete the episode

4. **Repeat steps 2-3 for more episodes**

5. **Stop the node when done:**
   Press `Ctrl+C` in the terminal running the node

---

## Tips

- Use absolute paths or run from workspace root for consistent output locations
- The controlled node logs the total number of saved episodes
- MCAP files can be viewed with [Foxglove Studio](https://foxglove.dev/)
- Each episode gets a unique timestamp to prevent overwrites
