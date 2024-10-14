FROM ghcr.io/otischung/pros_ai_image:1.3.6
ENV ROS2_WS=/workspaces
ENV ROS_DOMAIN_ID=1
ENV ROS_DISTRO=humble
ARG THREADS=4
ARG TARGETPLATFORM

##### Copy Source Code #####
COPY . /tmp

##### Environment Settings #####
WORKDIR ${ROS2_WS}

##### colcon Installation #####
# Copy source code
RUN mkdir -p ${ROS2_WS}/src && \
    mv /tmp/src/pros_car/src/pros_car_py ${ROS2_WS}/src/pros_car_py && \
    mv /tmp/src/ascamera ${ROS2_WS}/src/ascamera && \
    mv /tmp/src/yolov8/yolov8_msgs ${ROS2_WS}/src/yolov8_msgs && \
# Copy included files
    mv /tmp/include/subscribers.py /opt/ros/humble/local/lib/python3.10/dist-packages/rosbridge_library/internal/subscribers.py && \
    mv /tmp/include/scripts ${ROS2_WS}/scripts

### Ascamera Installation ###
RUN apt update && \
    apt install -y libpcl-dev && \
    mkdir -p ${ROS2_WS}/src/ros_perception && \
    git clone https://github.com/ros-perception/perception_pcl ${ROS2_WS}/src/ros_perception/perception_pcl && \
    git clone -b ros2 https://github.com/ros-perception/pcl_msgs ${ROS2_WS}/src/ros_perception/pcl_msgs && \
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select pcl_msgs --symlink-install && \
    colcon build --packages-select pcl_conversions --symlink-install && \
    colcon build --packages-select pcl_ros --symlink-install && \
    colcon build --packages-select perception_pcl --symlink-install

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --parallel-workers ${THREADS} --mixin release && \
##### Post-Settings #####
# Clear tmp and cache
    rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.bash"]
CMD ["bash", "-l"]
