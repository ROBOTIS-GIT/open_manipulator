FROM robotis/ros:jazzy-ros-base-librealsense AS librealsense

# Use the base ROS2 Jazzy image
FROM ros:jazzy-ros-base AS open-manipulator

ARG TARGETARCH
ARG S6_OVERLAY_VERSION=3.2.1.0

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install s6-overlay
RUN apt-get update && \
    apt-get install -y ca-certificates curl xz-utils && \
    rm -rf /var/lib/apt/lists/*

ADD https://github.com/just-containers/s6-overlay/releases/download/v${S6_OVERLAY_VERSION}/s6-overlay-noarch.tar.xz /tmp
RUN tar -C / -Jxpf /tmp/s6-overlay-noarch.tar.xz
RUN set -eux; \
    if [ -z "${TARGETARCH}" ]; then \
      echo "TARGETARCH is required. Build with docker buildx --platform linux/amd64 or linux/arm64." >&2; \
      exit 1; \
    fi; \
    case "${TARGETARCH}" in \
      amd64) s6_arch="x86_64" ;; \
      arm64) s6_arch="aarch64" ;; \
      *) echo "Unsupported TARGETARCH: ${TARGETARCH}" >&2; exit 1 ;; \
    esac; \
    curl -fsSL \
      -o /tmp/s6-overlay-${s6_arch}.tar.xz \
      "https://github.com/just-containers/s6-overlay/releases/download/v${S6_OVERLAY_VERSION}/s6-overlay-${s6_arch}.tar.xz"; \
    tar -C / -Jxpf /tmp/s6-overlay-${s6_arch}.tar.xz; \
    rm -f /tmp/s6-overlay-${s6_arch}.tar.xz

COPY --from=librealsense /opt/librealsense /usr/local/
COPY --from=librealsense /usr/local/lib/python3.12/dist-packages/pyrealsense2 /usr/local/lib/python3.12/dist-packages/pyrealsense2
COPY --from=librealsense /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib

# Install realsense dependencies
RUN apt-get update && apt-get install -y \
    libusb-1.0-0-dev \
    libglfw3-dev \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-usb-cam \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    vim \
    nano \
    python3-pip \
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

# Cap parallelism -- unbounded colcon build OOMs on hosts with many cores
# but limited RAM.
RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${COLCON_WS} && \
    MAKEFLAGS='-j4' colcon build --symlink-install --parallel-workers 4 --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo "alias omy_ai='ros2 launch open_manipulator_bringup omy_ai.launch.py'" >> ~/.bashrc && \
    echo "alias omx_ai='ros2 launch open_manipulator_bringup omx_ai.launch.py'" >> ~/.bashrc && \
    echo "alias zenohd='ros2 run rmw_zenoh_cpp rmw_zenohd'" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=30" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_zenoh_cpp" >> ~/.bashrc

# Create directory for agent Unix Domain Socket
RUN mkdir -p /var/run/agent && \
    chmod 777 /var/run/agent

RUN git clone https://github.com/ROBOTIS-GIT/cyclo_manager.git /opt/cyclo_manager

ENV PYTHONPATH=/opt/cyclo_manager:${PYTHONPATH}

RUN pip config set global.break-system-packages true && \
    pip install --no-cache-dir --ignore-installed -r /opt/cyclo_manager/requirements-agent.txt

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
COPY docker/assets/checkpoint-rs.tar /opt/graspnet-baseline/logs/log_rs/checkpoint-rs.tar

# For classifying/locating grasped objects
RUN pip3 install --break-system-packages --no-cache-dir google-genai

# New-style s6-rc services
COPY docker/s6-agent /etc/s6-overlay/s6-rc.d/s6-agent

ARG ROS2_SERVICES="open_manipulator_bringup leader_bringup"

COPY docker/s6-services/ /etc/s6-overlay/s6-rc.d/

RUN chmod +x /etc/s6-overlay/s6-rc.d/s6-agent/run /etc/s6-overlay/s6-rc.d/s6-agent/finish && \
    for service in ${ROS2_SERVICES}; do \
    chmod +x /etc/s6-overlay/s6-rc.d/${service}/run /etc/s6-overlay/s6-rc.d/${service}/finish && \
    chmod +x /etc/s6-overlay/s6-rc.d/${service}-log/run && \
    mkdir -p /var/log/${service} && \
    chown nobody:nogroup /var/log/${service} && \
    chmod 02755 /var/log/${service}; \
    done && \
    chmod +x /etc/s6-overlay/s6-rc.d/rebuild_omy_playground/up && \
    rm -rf /etc/s6-overlay/s6-rc.d/common

RUN mkdir -p /usr/local/lib/s6-services
COPY docker/s6-services/common/ros2_service_run.sh /usr/local/lib/s6-services/ros2_service_run.sh
COPY docker/s6-services/common/ros2_service_finish.sh /usr/local/lib/s6-services/ros2_service_finish.sh
COPY docker/s6-services/common/rebuild_omy_playground.sh /usr/local/lib/s6-services/rebuild_omy_playground.sh
RUN chmod +x /usr/local/lib/s6-services/ros2_service_run.sh /usr/local/lib/s6-services/ros2_service_finish.sh \
    /usr/local/lib/s6-services/rebuild_omy_playground.sh

# Start the agent and the playground rebuild (see rebuild_omy_playground/up)
# by default. ROS2 services can be managed through s6.
RUN mkdir -p /etc/s6-overlay/s6-rc.d/user/contents.d && \
    touch /etc/s6-overlay/s6-rc.d/user/contents.d/s6-agent && \
    touch /etc/s6-overlay/s6-rc.d/user/contents.d/rebuild_omy_playground

ENTRYPOINT ["/init"]
