#!/bin/bash
# Latency / real-time check inside the container: play a bag at the sensor rate through the node with
# the given parameters and record every processed frame (record_path JSONL: latency_ms, dropped,
# frames, alerts ...).
#   live_check.sh <bag dir> <record.jsonl> [rate]
# env NODE_ARGS   extra node parameters, e.g. "-p max_queue:=100 -p use_stamps:=false" (the pre-v7 policy)
#     QOS=1       replay with the lossless QoS override (only the node's own policy drops frames)
#     ROS_DOMAIN_ID (default 57) + localhost only: isolated from other ROS processes on the host
set -eo pipefail
BAG=$1; OUT=$2; RATE=${3:-1}
HERE=$(cd "$(dirname "$0")" && pwd)
. "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
. /opt/metro/ws/install/setup.bash
export METRO_LIB=${METRO_LIB:-/opt/metro/lib/libmetro_core.so}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-57} ROS_LOCALHOST_ONLY=1
rm -f "$OUT"
PLAY=()
[ "${QOS:-0}" = 1 ] && PLAY=(--qos-profile-overrides-path "$HERE/qos_replay.yaml")
ros2 run metro_detector metro_detector --ros-args -p record_path:="$OUT" $NODE_ARGS &
NODE=$!
sleep 8
ros2 bag play "$BAG" --rate "$RATE" "${PLAY[@]}" --disable-keyboard-controls
prev=-1
while true; do
    n=$(wc -l < "$OUT" 2>/dev/null || echo 0)
    [ "$n" = "$prev" ] && break
    prev=$n; sleep 5
done
pkill -TERM -f "lib/metro_detector/metro_detector" || true
kill -TERM $NODE 2> /dev/null || true
wait $NODE || true
echo "LIVE_CHECK_DONE $(wc -l < "$OUT") frames -> $OUT"
