FROM robotis/ros:jazzy-ros-base-librealsense AS librealsense

# Use the base ROS2 Jazzy image
FROM ros:jazzy-ros-base AS open-manipulator

COPY --from=librealsense /opt/librealsense /usr/local/
COPY --from=librealsense /usr/local/lib/python3.12/dist-packages/pyrealsense2 /usr/local/lib/python3.12/dist-packages/pyrealsense2
COPY --from=librealsense /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install realsense dependencies
RUN apt-get update && apt-get install -y \
    libusb-1.0-0-dev \
    libglfw3-dev \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

ENV COLCON_WS=/root/ros2_ws
WORKDIR ${COLCON_WS}

RUN mkdir -p ${COLCON_WS}/src && \
    cd ${COLCON_WS}/src && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/open_manipulator.git && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/robotis_interfaces.git && \
    git clone -b r/4.56.4 https://github.com/IntelRealSense/realsense-ros.git && \
    cd realsense-ros && git checkout cbd8b8487d4c0126bb590822db9469414496e62a

RUN cd ${COLCON_WS} && \
    apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="librealsense2 robotis_interfaces" -y && \
    rm -rf /var/lib/apt/lists/*

# Capped: colcon's default (one job per core) can OOM-kill the whole host on
# machines with many cores but limited RAM (confirmed on a 20-core/15GB host).
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    MAKEFLAGS='-j4' colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='MAKEFLAGS=-j4 colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo "alias omy_ai='ros2 launch open_manipulator_bringup omy_ai.launch.py'" >> ~/.bashrc && \
    echo "alias omx_ai='ros2 launch open_manipulator_bringup omx_ai.launch.py'" >> ~/.bashrc && \
    echo "alias zenohd='ros2 run rmw_zenoh_cpp rmw_zenohd'" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc

# Install CUDA toolkit
RUN apt-get update && apt-get install -y --no-install-recommends \
        wget gnupg python3-pip wmctrl python3-tk \
    && wget -qO /tmp/cuda-keyring.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb \
    && apt-get update && apt-get install -y --no-install-recommends cuda-toolkit-12-8 \
    && rm -rf /var/lib/apt/lists/*

ENV PATH=/usr/local/cuda-12.8/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12.8/lib64:${LD_LIBRARY_PATH}
# Target GPU architectures for pointnet2/knn
ENV TORCH_CUDA_ARCH_LIST="7.5;8.6;8.9"

# Install torch (CUDA 12.8 build)
RUN pip3 install --break-system-packages --no-cache-dir --ignore-installed ninja
RUN pip3 install --break-system-packages --no-cache-dir --ignore-installed \
        torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128

# Install graspnet-baseline
RUN git clone https://github.com/graspnet/graspnet-baseline.git /opt/graspnet-baseline && \
    cd /opt/graspnet-baseline && \
    sed -i '/^torch/d' requirements.txt && \
    pip3 install --break-system-packages --no-cache-dir --ignore-installed -r requirements.txt && \
    sed -i 's/from torch._six import container_abcs/import collections.abc as container_abcs/' dataset/graspnet_dataset.py && \
    cd pointnet2 && python3 setup.py install && \
    cd ../knn && python3 setup.py install

RUN git clone https://github.com/graspnet/graspnetAPI.git /opt/graspnetAPI && \
    pip3 install --break-system-packages --no-cache-dir --ignore-installed /opt/graspnetAPI

# Pretrained checkpoint (RealSense model), from graspnet-baseline's release
COPY assets/checkpoint-rs.tar /opt/graspnet-baseline/logs/log_rs/checkpoint-rs.tar

# For classifying/locating grasped objects
RUN pip3 install --break-system-packages --no-cache-dir google-genai

CMD ["bash"]
