#!/bin/bash
source /opt/ros/humble/setup.bash
colcon build
. ./install/setup.bash
ros2 run pros_car_py carC_writer &
ros2 run pros_car_py arm_writer &
ros2 launch /workspace/demo/rplidar.xml &
ros2 launch /workspace/demo/slam.xml &
ros2 launch /workspace/demo/navigation.xml &
ros2 run image_transport republish raw --ros-args -r in:=yolo/image -r out/compressed:=camera/yolo/compressed -p image_transport:=compressed &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
tail -f /dev/null
