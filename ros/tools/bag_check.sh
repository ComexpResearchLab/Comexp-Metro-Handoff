#!/bin/bash
# Offline check, inside the container: play a bag through the node and record every frame.
#   bag_check.sh <bag dir> <record.jsonl> [rate] [engine library]
# Default (check mode): the node runs with reliable deep queues (no frame dropped) and the bag plays slowly
# enough (rate 0.25) that no frame is dropped; the record then has one line per bag message
# (the per-frame records can be compared across runs with compare_alerts.py).
# LIVE=1: the node's default settings and the bag's own QoS -- use with rate 1 to measure latency
# and dropped frames at the sensor rate.
set -eo pipefail
BAG=$1; OUT=$2; RATE=${3:-0.25}; LIB=${4:-}
HERE=$(cd "$(dirname "$0")" && pwd)
. "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
[ -f /opt/metro/ws/install/setup.bash ] && . /opt/metro/ws/install/setup.bash
rm -f "$OUT"
if [ -n "$LIVE" ]; then
    QP=(); PLAY=()
else
    QP=(-p qos_reliable:=true -p qos_depth:=200 -p max_queue:=200 -p warmup_queue:=200); PLAY=(--qos-profile-overrides-path "$HERE/qos_replay.yaml")
fi
ros2 run metro_detector metro_detector --ros-args -p record_path:="$OUT" "${QP[@]}" ${LIB:+-p library:="$LIB"} &
NODE=$!
sleep 8
ros2 bag play "$BAG" --rate "$RATE" "${PLAY[@]}" --disable-keyboard-controls
# wait until the node has drained its queue (record stops growing)
prev=-1
while true; do
    n=$(wc -l < "$OUT" 2>/dev/null || echo 0)
    [ "$n" = "$prev" ] && break
    prev=$n; sleep 10
done
# (a background job of a non-interactive shell ignores SIGINT: stop the node with SIGTERM)
pkill -TERM -f "lib/metro_detector/metro_detector" || true
kill -TERM $NODE 2> /dev/null || true
wait $NODE || true
echo "BAG_CHECK_DONE $(wc -l < "$OUT") frames -> $OUT"
