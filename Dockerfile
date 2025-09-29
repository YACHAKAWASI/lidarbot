# ================================
#  LIDARBOT — ROS 2 Humble (Jammy)
#  Single-stage (build = runtime)
# ================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    ROS_DOMAIN_ID=10 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PACKAGES_SKIP="lidarbot_gazebo lidarbot_gz"

# Herramientas de compilación y ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git pkg-config \
      python3-pip python3-colcon-common-extensions \
      python3-rosdep python3-vcstool rsync \
    && rm -rf /var/lib/apt/lists/*

# rosdep
RUN rosdep init || true && rosdep update

# Workspace
WORKDIR /ws

# Copiamos el repo completo (usa .dockerignore para no subir basura al build)
COPY . /ws/src_root/

# Sincronizamos solo src/ al workspace
# RUN mkdir -p /ws/src && rsync -a --delete /ws/src_root/src/ /ws/src/ || true
# Copiamos TODO el repo al workspace, excluyendo lo que no es código ROS
RUN mkdir -p /ws/src && rsync -a --delete \
    --exclude '.git' --exclude '.github' --exclude 'docker' --exclude 'docs' \
    /ws/src_root/ /ws/src/


# Dependencias de sistema declaradas en package.xml
RUN apt-get update && \
    rosdep install --from-paths /ws/src --rosdistro ${ROS_DISTRO} -y --ignore-src \
      --skip-keys="rti-connext-dds-6.0.1" || true && \
    rm -rf /var/lib/apt/lists/*

# Build
SHELL ["/bin/bash", "-lc"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 --event-handlers console_direct+ \
                 --packages-skip ${PACKAGES_SKIP}

# Entrypoint + comando por defecto
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

ENV APP_LAUNCH_PKG=lidarbot_bringup \
    APP_LAUNCH_FILE=lidarbot_bringup_launch.py
CMD ["ros2", "launch", "lidarbot_bringup", "lidarbot_bringup_launch.py"]
