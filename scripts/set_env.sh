export ROS_DOMAIN_ID=100

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export NETWORK_INTERFACE=enp193s0f3u1
export CYCLONEDDS_URI=file:///home/daniel/repos/bootcamp_ws/scripts/cyclone_config.xml

ros2 daemon stop && ros2 daemon start
