#!/usr/bin/env python3
"""The node's queueing at the sensor rate without ROS: a feeder thread delivers a bag's clouds at their
header-stamp pace (10 Hz), then the node's own two stages run exactly as in node.py -- a prep thread
(decode + regrid, metro_detector.cloud) and an engine thread (metro_detector.engine) -- with the node's
queue policy. Measures what the queue policy does to latency, drops, odometry and alerts; excludes DDS.

  node_sim.py <bag.db3> <out.jsonl> [--max-queue 2] [--warmup-queue 10] [--no-stamps] [--lib LIB] [--python-regrid]
  (the pre-v7 node: --max-queue 100 --warmup-queue 100 --no-stamps --python-regrid)
One JSON line per processed frame, the node's record format (stamp, s, ds, locked, event, frames, alerts,
latency_ms {decode, queue, engine, total}, dropped).
"""
import argparse
import json
import os
import sqlite3
import struct
import sys
import threading
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'metro_detector'))
from metro_detector.cloud import engine_frame, make_regridder, to_array   # noqa: E402
from metro_detector.engine import Engine                                   # noqa: E402
from engine_bench import parse_pointcloud2                                 # noqa: E402


def header_stamp(data):
    sec, nsec = struct.unpack_from('<iI', data, 4)
    return sec + nsec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag'); ap.add_argument('out')
    ap.add_argument('--max-queue', type=int, default=1)
    ap.add_argument('--warmup-queue', type=int, default=20)
    ap.add_argument('--max-skip', type=float, default=0.5, help='node max_skip (s); 0 = plain drop-oldest (before)')
    ap.add_argument('--no-stamps', action='store_true')
    ap.add_argument('--python-regrid', action='store_true')
    ap.add_argument('--lib', default=None)
    o = ap.parse_args()
    eng = Engine(o.lib)
    rg = None if o.python_regrid else make_regridder(o.lib)
    raw, queue = deque(), deque()
    pcv, cv = threading.Condition(), threading.Condition()
    st = dict(locked=False, dropped=0, done=False, last_prep=None, last_eng=None)
    HARD = 30

    def take(q, last):                                   # node.take
        b = bound()
        while len(q) > b:
            if o.max_skip > 0 and last is not None and q[1] is not None and q[1][1] - last > o.max_skip:
                break
            q.popleft(); st['dropped'] += 1
        return q.popleft()
    bound = lambda: max(1, o.max_queue if st['locked'] else max(o.max_queue, o.warmup_queue))
    out = open(o.out, 'w')

    def feeder():
        con = sqlite3.connect(o.bag)
        rows = con.execute("select m.data from messages m join topics t on m.topic_id = t.id "
                           "where t.type = 'sensor_msgs/msg/PointCloud2' order by m.timestamp")
        t_start, s0 = None, None
        for (data,) in rows:
            s = header_stamp(data)
            if t_start is None:
                t_start, s0 = time.perf_counter(), s
            wait = t_start + (s - s0) - time.perf_counter()
            if wait > 0:
                time.sleep(wait)
            with pcv:                                    # node.on_cloud
                if len(raw) >= (HARD if o.max_skip > 0 else bound()):
                    raw.popleft(); st['dropped'] += 1
                raw.append((data, s, time.perf_counter()))
                pcv.notify()
        with pcv:
            st['done'] = True; pcv.notify_all()

    def prep():                                          # node.prep
        while True:
            with pcv:
                while not raw and not st['done']:
                    pcv.wait(0.5)
                if not raw and st['done']:
                    with cv:
                        queue.append(None); cv.notify()
                    return
                data, stamp, t0 = take(raw, st['last_prep']) if o.max_skip > 0 else raw.popleft()
                st['last_prep'] = stamp
            pc = parse_pointcloud2(data)
            a = to_array(pc['fields'], pc['point_step'], pc['data'], bool(pc['is_bigendian']))
            frame, regridded, _ = engine_frame(a, stamp=stamp, regridder=rg)
            t1 = time.perf_counter()
            with cv:
                if len(queue) >= (HARD if o.max_skip > 0 else bound()):
                    queue.popleft(); st['dropped'] += 1
                queue.append((frame, stamp, t0, t1))          # the stamp at [1], as in the node
                cv.notify()

    th = [threading.Thread(target=feeder, daemon=True), threading.Thread(target=prep, daemon=True)]
    for t in th:
        t.start()
    k = 0
    while True:                                          # node.worker + process
        with cv:
            while not queue:
                cv.wait(0.5)
            if queue[0] is None:
                break
            if o.max_skip > 0:
                item = take(queue, st['last_eng'])
            else:
                while len(queue) > bound() and queue[1] is not None:
                    queue.popleft(); st['dropped'] += 1
                item = queue.popleft()
            if item is None:
                break
            st['last_eng'] = item[1]
        frame, stamp, t0, t1 = item
        t1b = time.perf_counter()
        r = eng.step(frame, verbose=True, stamp=None if o.no_stamps else stamp)
        t2 = time.perf_counter()
        od = r.get('odo') or {}
        st['locked'] = bool(od.get('locked', False))
        lat = {'decode': round((t1 - t0) * 1e3, 2), 'queue': round((t1b - t1) * 1e3, 2), 'engine': round((t2 - t1b) * 1e3, 2),
               'total': round((t2 - t0) * 1e3, 2)}
        out.write(json.dumps({'k': k, 'stamp': stamp, 's': r.get('s'), 'ds': r.get('ds'), 'locked': st['locked'], 'event': od.get('event'),
                              'frames': od.get('frames'), 'alerts': r['alerts'], 'latency_ms': lat, 'dropped': st['dropped']}) + '\n')
        k += 1
    out.close()
    print(f'{k} frames processed, {st["dropped"]} dropped -> {o.out}')


if __name__ == '__main__':
    main()
