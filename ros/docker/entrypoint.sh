#!/bin/bash
# Container entry point: ROS 2 + the metro_detector workspace, then the command.
set -e
# Delivery mode: METRO_LIB is intentionally unset so the node unlocks the machine-bound engine
# (activation) into memory at start-up. Only a developer/parity run sets METRO_LIB to a plaintext
# .so, in which case we honour it and skip activation.
if [ -n "${METRO_LIB:-}" ]; then
    export METRO_LIB
fi
# optional large-message Fast DDS profile (METRO_FASTDDS_LARGE=1); off by default: on our host it
# did not remove the frame losses of a replay without the QoS override, see README
if [ "${METRO_FASTDDS_LARGE:-0}" = 1 ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/metro/fastdds_large.xml
fi
. "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
. /opt/metro/ws/install/setup.bash
exec "$@"
