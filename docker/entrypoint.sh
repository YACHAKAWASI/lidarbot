#!/usr/bin/env bash
set -e

# Entornos ROS
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash || true
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

# Defaults útiles
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-10}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}

# Si definiste APP_LAUNCH_PKG/FILE, priorízalo
if [[ -n "${APP_LAUNCH_PKG}" && -n "${APP_LAUNCH_FILE}" ]]; then
  set -- ros2 launch "${APP_LAUNCH_PKG}" "${APP_LAUNCH_FILE}"
fi

exec "$@"
