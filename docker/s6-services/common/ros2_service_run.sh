#!/bin/bash
# Reusable ROS2 service run script.

set -e

SERVICE_NAME="${SERVICE_NAME}"
if [ -z "${SERVICE_NAME}" ]; then
    echo "Error: SERVICE_NAME environment variable must be set" >&2
    exit 1
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
export COLCON_WS=${COLCON_WS:-/root/ros2_ws}
export S6_VERBOSITY=${S6_VERBOSITY:-1}

echo "[${SERVICE_NAME}] Starting service..."
echo "[${SERVICE_NAME}] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[${SERVICE_NAME}] ROS_DISTRO=${ROS_DISTRO}"
echo "[${SERVICE_NAME}] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "[${SERVICE_NAME}] COLCON_WS=${COLCON_WS}"
echo "[${SERVICE_NAME}] PID: $$"

PGID=$(ps -o pgid= -p $$ | tr -d ' ')
echo "[${SERVICE_NAME}] Process group: ${PGID}"
echo "${PGID}" > /run/${SERVICE_NAME}.pgid || true

source /opt/ros/${ROS_DISTRO}/setup.bash
source ${COLCON_WS}/install/setup.bash

if [ -n "${ROS2_COMMAND}" ]; then
    ROS2_CMD="${ROS2_COMMAND}"
    echo "[${SERVICE_NAME}] Executing custom command: ${ROS2_CMD}"
else
    ROS2_CMD="ros2 launch open_manipulator_bringup ${SERVICE_NAME}.launch.py"
    echo "[${SERVICE_NAME}] Executing default command: ${ROS2_CMD}"
fi

LAUNCH_ARGS_FILE="/run/launch_args/${SERVICE_NAME}"
if [ -f "${LAUNCH_ARGS_FILE}" ]; then
    LAUNCH_ARGS=$(cat "${LAUNCH_ARGS_FILE}")
    ROS2_CMD="${ROS2_CMD} ${LAUNCH_ARGS}"
    echo "[${SERVICE_NAME}] Launch args: ${LAUNCH_ARGS}"
fi

exec bash -i -c "${ROS2_CMD}"
