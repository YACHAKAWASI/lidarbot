# ================================
#  LIDARBOT — ROS 2 Humble (Jammy)
# ================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=10 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PACKAGES_SKIP="lidarbot_gazebo lidarbot_gz"

# Herramientas + deps de sistema y binarios ROS usados por lidarbot_* (OpenCV/imagen/tf2)
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config rsync curl ca-certificates \
      python3-pip python3-colcon-common-extensions \
      python3-rosdep python3-vcstool \
      libi2c-dev i2c-tools \
      libopencv-dev python3-opencv \
      libeigen3-dev libyaml-cpp-dev \
      ros-humble-cv-bridge \
      ros-humble-image-transport \
      ros-humble-camera-info-manager \
      ros-humble-tf2-ros ros-humble-tf2-eigen \
      ros-humble-rclcpp ros-humble-rclcpp-components \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /ws
COPY . /ws/src_root/
RUN mkdir -p /ws/src && rsync -a --delete \
      --exclude '.git' --exclude '.github' --exclude 'docker' --exclude 'docs' \
      /ws/src_root/ /ws/src/

RUN apt-get update && \
    rosdep install --from-paths /ws/src --rosdistro ${ROS_DISTRO} -y --ignore-src \
      --skip-keys="rti-connext-dds-6.0.1" || true && \
    rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-lc"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 --event-handlers console_direct+ \
                 --packages-skip ${PACKAGES_SKIP}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

ENV APP_LAUNCH_PKG=lidarbot_bringup \
    APP_LAUNCH_FILE=lidarbot_bringup_launch.py
CMD ["ros2", "launch", "lidarbot_bringup", "lidarbot_bringup_launch.py"]
