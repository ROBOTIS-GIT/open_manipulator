#!/bin/bash
# The open_manipulator packages are colcon-built once at image build time,
# using whatever the Dockerfile's git clone fetched -- then docker-compose
# bind-mounts the actual local checkout over that same source path at
# container start. colcon's incremental build cache doesn't notice that
# swap, so a plain rebuild can silently skip packages that changed. Force a
# clean rebuild of just those packages (not the whole workspace) here.
set -e
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
export COLCON_WS=${COLCON_WS:-/root/ros2_ws}
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${COLCON_WS}"

OM_PACKAGES="open_manipulator open_manipulator_bringup open_manipulator_collision open_manipulator_description open_manipulator_gui open_manipulator_moveit_config open_manipulator_playground open_manipulator_teleop"

for pkg in ${OM_PACKAGES}; do
    rm -rf "build/${pkg}" "install/${pkg}"
done

# Cap parallelism -- unbounded colcon build OOMs on hosts with many cores
# but limited RAM.
MAKEFLAGS='-j4' colcon build --symlink-install --parallel-workers 4 --packages-select ${OM_PACKAGES} --cmake-args -DCMAKE_BUILD_TYPE=Release
