#!/usr/bin/env python3
"""Time the node's per-frame work without ROS: decode + (regrid) + engine step, on a bag file.

  python3 engine_bench.py <bag.db3> [--limit N] [--lib libmetro_core.so]

Uses the node's own modules (metro_detector.cloud / .engine) and a minimal rosbag2 sqlite + CDR
reader (a minimal PointCloud2 decoder), so it runs anywhere the node runs (inside the image:
python3 /opt/metro/tools/engine_bench.py /bags/x/x_0.db3).
"""
import argparse
import os
import sqlite3
import struct
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'metro_detector'))
from metro_detector.cloud import engine_frame, to_array   # noqa: E402
from metro_detector.engine import Engine                   # noqa: E402


def parse_pointcloud2(buf):
    """CDR sensor_msgs/PointCloud2 decoder."""
    o = 4

    def al(n):
        nonlocal o
        o = ((o - 4 + n - 1) // n) * n + 4

    def u32():
        nonlocal o
        al(4); v = struct.unpack_from('<I', buf, o)[0]; o += 4; return v

    def i32():
        nonlocal o
        al(4); v = struct.unpack_from('<i', buf, o)[0]; o += 4; return v

    def u8():
        nonlocal o
        v = buf[o]; o += 1; return v

    def string():
        nonlocal o
        n = u32(); s = buf[o:o + n - 1].decode(); o += n; return s
    sec = i32(); nsec = u32(); frame_id = string()
    height = u32(); width = u32()
    fields = []
    for _ in range(u32()):
        name = string(); off = u32(); dt = u8(); cnt = u32(); fields.append((name, off, dt, cnt))
    big = u8(); point_step = u32(); row_step = u32()
    n = u32(); data = buf[o:o + n]
    return dict(stamp=sec + nsec * 1e-9, frame_id=frame_id, fields=fields, point_step=point_step, data=data, is_bigendian=big)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--limit', type=int, default=None)
    ap.add_argument('--lib', default=None)
    o = ap.parse_args()
    eng = Engine(o.lib)
    con = sqlite3.connect(o.bag)
    q = 'select data from messages order by timestamp' + (f' limit {o.limit}' if o.limit else '')
    td, te, alerts = [], [], 0
    for (data,) in con.execute(q):
        pc = parse_pointcloud2(data)
        t0 = time.perf_counter()
        a = to_array(pc['fields'], pc['point_step'], pc['data'], bool(pc['is_bigendian']))
        fr, _, _ = engine_frame(a, stamp=pc['stamp'])
        t1 = time.perf_counter()
        r = eng.step(fr)
        t2 = time.perf_counter()
        td.append((t1 - t0) * 1e3); te.append((t2 - t1) * 1e3); alerts += bool(r['alerts'])
    p = lambda v: 'p50 %.1f p90 %.1f p99 %.1f' % tuple(np.percentile(v, [50, 90, 99]))
    print(f'frames {len(te)}  alert frames {alerts}')
    print(f'decode+regrid ms: {p(td)}')
    print(f'engine ms:        {p(te)}')
    print(f'sum ms:           {p(np.add(td, te))}')


if __name__ == '__main__':
    main()
