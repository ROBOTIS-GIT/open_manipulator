# Use the base ROS2 Jazzy image
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Define build argument for Gazebo support
ARG WITH_GAZEBO=true

# Install common system dependencies
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-pal-statistics \
    vim \
    && apt-get install -y ros-${ROS_DISTRO}-moveit-* --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo dependencies conditionally
RUN if [ "$WITH_GAZEBO" = "true" ]; then \
    apt-get update && apt-get install -y ros-${ROS_DISTRO}-gz* \
    && rm -rf /var/lib/apt/lists/*; \
    fi

ENV COLCON_WS=/root/colcon_ws
WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git

# Clone gz_ros2_control only if WITH_GAZEBO is true
RUN if [ "$WITH_GAZEBO" = "true" ]; then \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ros-controls/gz_ros2_control; \
    fi

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc  && \
    echo "export ROBOT_MODEL=om_y_follower" >> ~/.bashrc

CMD ["bash"]
