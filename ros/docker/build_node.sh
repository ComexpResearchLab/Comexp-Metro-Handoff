#!/bin/bash
# Build the ROS 2 package with colcon.
#   build_node.sh <workspace dir>      (the package is in <workspace>/src/metro_detector)
set -eo pipefail
WS=$1
. "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "$WS"
colcon build --merge-install --event-handlers console_direct+
