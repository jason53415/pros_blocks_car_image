#!/bin/bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
. ./install/setup.bash
ros2 launch /workspace/demo/rplidar_unity.xml &
ros2 launch /workspace/demo/slam.xml &
ros2 launch /workspace/demo/navigation.xml &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
tail -f /dev/null
