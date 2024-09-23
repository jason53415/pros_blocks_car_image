#!/bin/bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
. ./install/setup.bash
ros2 run pros_car_py carB_writer &
# ros2 run pros_car_py arduino_arm_writer &
ros2 launch /workspaces/scripts/demo/ydlidar.py.launch &
ros2 launch /workspaces/scripts/demo/slam.xml &
ros2 launch /workspaces/scripts/demo/navigation.xml &
ros2 launch ascamera hp60c.launch.py &
ros2 run image_transport republish raw --ros-args -r in:=ascamera_hp60c/camera_publisher/rgb0/image -r out/compressed:=camera/rgb/compressed -p image_transport:=compressed &
ros2 run image_transport republish raw --ros-args -r in:=ascamera_hp60c/camera_publisher/depth0/image_raw -r out/compressedDepth:=camera/depth/compressed -p image_transport:=compressed_depth &
ros2 run image_transport republish raw --ros-args -r in:=yolo/image -r out/compressed:=camera/yolo/compressed -p image_transport:=compressed &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
tail -f /dev/null
