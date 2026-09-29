#!/usr/bin/env python3
"""Compare the node's per-frame record with the reference engine run, and summarise latency.

  python3 compare_alerts.py <node.jsonl> <reference.jsonl> [--json out.json]

Frames are matched by (topic, header stamp). Equal = in every frame the same set of alerting
objects (id), each with the same integer fields (n, hits, tag) and float fields (d, lateral,
height, first_alert_d, int_med) within FLOAT_TOL, the same odometry lock state and s / ds within
FLOAT_TOL, and no frame missing on either side. Exit status 0 when equal.

Why a tolerance: libmetro_core.so calls the system libm (sin, cos, atan2, exp, ...) dynamically.
The reference runs on the host (Ubuntu 20.04, glibc 2.31) and the node in the ROS image (Ubuntu
22.04, glibc 2.35), whose libm differs in the last bit of some results; this shows as ~1e-14 m in
the odometry and in the alert distances. Measured: the SAME binary gives bit-identical output on
the host with 4, 8 and 24 threads, and ulp-level differences only between host and container.
Bit-identical frames are counted separately (alerts_bit_identical_frames).
"""
import argparse
import json
import math
import sys

import numpy as np

FLOAT_TOL = 1e-6      # m
FLOATS = ('d', 'lateral', 'height', 'first_alert_d', 'int_med')
INTS = ('n', 'hits', 'tag')


def load(fn):
    out = {}
    for line in open(fn):
        line = line.strip()
        if line:
            r = json.loads(line)
            out[(r['topic'], r['stamp'])] = r
    return out


def fdiff(a, b):
    if a is None or b is None:
        return 0.0 if a is None and b is None else math.inf
    return abs(a - b)


def alerts_diff(xa, xb):
    """-> (same structure, max float difference)"""
    A = {q['id']: q for q in xa}
    B = {q['id']: q for q in xb}
    if set(A) != set(B):
        return False, math.inf
    dm = 0.0
    for i in A:
        if any(A[i].get(k) != B[i].get(k) for k in INTS):
            return False, math.inf
        dm = max([dm] + [fdiff(A[i].get(k), B[i].get(k)) for k in FLOATS])
    return True, dm


def first_alerts(recs):
    seen = {}
    for r in sorted(recs.values(), key=lambda q: q['k']):
        for a in r['alerts']:
            if a['id'] not in seen:
                seen[a['id']] = dict(k=r['k'], d=a['d'], lateral=a['lateral'], height=a['height'])
    return seen


def pct(v, p):
    return round(float(np.percentile(v, p)), 1) if len(v) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('node')
    ap.add_argument('ref')
    ap.add_argument('--json', default=None)
    o = ap.parse_args()
    A, B = load(o.node), load(o.ref)
    keys_a, keys_b = set(A), set(B)
    common = sorted(keys_a & keys_b, key=lambda k: B[k]['k'])
    mism_alert, mism_odo, bit_alert, bit_odo, first_bad = 0, 0, 0, 0, None
    amax, omax = 0.0, 0.0
    for k in common:
        a, b = A[k], B[k]
        same, dm = alerts_diff(a['alerts'], b['alerts'])
        bit_alert += sorted(map(lambda q: json.dumps(q, sort_keys=True), a['alerts'])) == \
            sorted(map(lambda q: json.dumps(q, sort_keys=True), b['alerts']))
        if same:
            amax = max(amax, dm)
        if not same or dm > FLOAT_TOL:
            mism_alert += 1
            first_bad = first_bad or (k, a['alerts'], b['alerts'])
        do = max(fdiff(a['s'], b['s']), fdiff(a['ds'], b['ds']))
        bit_odo += (a['s'], a['ds'], a['locked']) == (b['s'], b['ds'], b['locked'])
        if a['locked'] == b['locked']:
            omax = max(omax, do)
        if a['locked'] != b['locked'] or do > FLOAT_TOL:
            mism_odo += 1
            first_bad = first_bad or (k, (a['s'], a['ds'], a['locked']), (b['s'], b['ds'], b['locked']))
    fa, fb = first_alerts(A), first_alerts(B)
    first_same = set(fa) == set(fb) and all(fa[i]['k'] == fb[i]['k'] and fdiff(fa[i]['d'], fb[i]['d']) <= FLOAT_TOL for i in fb)
    lat = {q: [r['latency_ms'][q] for r in A.values() if r.get('latency_ms') and r['latency_ms'].get(q) is not None]
           for q in ('decode', 'queue', 'engine', 'odometry', 'publish', 'total')}
    res = dict(
        node_frames=len(A), ref_frames=len(B), common=len(common),
        missing_in_node=len(keys_b - keys_a), extra_in_node=len(keys_a - keys_b),
        alert_mismatch_frames=mism_alert, odometry_mismatch_frames=mism_odo,
        alerts_bit_identical_frames=bit_alert, odometry_bit_identical_frames=bit_odo,
        alert_max_abs_diff_m=amax, odometry_max_abs_diff_m=omax,
        alert_frames=sum(bool(B[k]['alerts']) for k in common), objects=len(fb),
        first_alerts_ref={str(i): v for i, v in fb.items()}, first_alerts_equal=first_same,
        regrid_frames=sum(bool(r.get('regrid')) for r in A.values()),
        latency_ms={q: dict(p50=pct(v, 50), p90=pct(v, 90), p99=pct(v, 99), max=pct(v, 100), mean=round(float(np.mean(v)), 1) if v else None)
                    for q, v in lat.items()},
    )
    res['equal'] = bool(res['missing_in_node'] == 0 and res['extra_in_node'] == 0 and mism_alert == 0 and mism_odo == 0)
    print(json.dumps({k: v for k, v in res.items() if k != 'first_alerts_ref'}, indent=1))
    print('first alerts (reference):')
    for i, v in sorted(fb.items(), key=lambda q: q[1]['k']):
        print(f"  #{i}: frame {v['k']}  d {v['d']:.2f} m  lateral {v['lateral']:+.2f}  height {v['height']:.2f}")
    if first_bad:
        print('FIRST MISMATCH', first_bad)
    print('EQUAL' if res['equal'] else 'NOT EQUAL')
    if o.json:
        json.dump(res, open(o.json, 'w'), indent=1)
    sys.exit(0 if res['equal'] else 1)


if __name__ == '__main__':
    main()
