"""ROS 2 node: Hesai OT128 PointCloud2 -> foreign-object alerts ahead of the train.

Subscribes to one or more lidar topics (one engine / tracker per topic), runs the Rust engine on
every frame (internal odometry: ds = NaN), and publishes

  <alerts_topic>     std_msgs/String   JSON per frame: obstacle yes/no, nearest distance, every alerting
                                       object (distance ahead, lateral offset from the track centre,
                                       height over the rail-head line, ...), odometry, latency
  <obstacle_topic>   std_msgs/Bool     obstacle present in this frame
  <distance_topic>   std_msgs/Float32  distance (m) to the nearest alerting object, -1 when none
  <markers_topic>    visualization_msgs/MarkerArray   boxes + labels at the objects, track centreline
  <status_topic>     std_msgs/String   JSON: odometry lock, frame time, rates, per-stream counters
  /diagnostics       diagnostic_msgs/DiagnosticArray  (1 Hz)

Parameters: see declare_parameter calls below and ros/README.md.
"""
import json
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
try:                                                    # Humble
    from rclpy.qos_event import SubscriptionEventCallbacks
except ImportError:                                     # Iron and later
    try:
        from rclpy.event_handler import SubscriptionEventCallbacks
    except ImportError:
        SubscriptionEventCallbacks = None
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .cloud import engine_frame, make_regridder, msg_to_array
from .engine import Engine


def _r(v, nd=3):
    return None if v is None else round(float(v), nd)


class Stream:
    """Per input topic: its own tracker and counters."""

    def __init__(self, topic, lib_path, rust_regrid=True):
        self.topic = topic
        self.engine = Engine(lib_path)
        self.regridder = make_regridder(lib_path) if rust_regrid else None
        self.dropped_logged = 0
        self.gaps = 0
        self.last_prep = None                           # stamps of the last frame each stage took
        self.last_eng = None
        self.k = 0
        self.seen = set()
        self.locked = False
        self.last = None
        self.t_total = deque(maxlen=200)
        self.t_arrival = deque(maxlen=50)
        self.alert_frames = 0
        self.dropped = 0
        self.queue = deque()
        self.cv = threading.Condition()
        self.thread = None
        self.raw = deque()                              # received messages, not decoded yet
        self.pcv = threading.Condition()
        self.pthread = None


class MetroDetector(Node):

    def __init__(self):
        super().__init__('metro_detector')
        P = self.declare_parameter
        self.p_inputs = list(P('input_topics', ['/lidar_points', '/sensing/lidar/hesai128/pointcloud']).value)
        self.p_alerts = P('alerts_topic', '/metro/alerts').value
        self.p_obst = P('obstacle_topic', '/metro/obstacle').value
        self.p_dist = P('distance_topic', '/metro/obstacle_distance').value
        self.p_markers = P('markers_topic', '/metro/markers').value
        self.p_status = P('status_topic', '/metro/status').value
        self.p_frame = P('frame_id', '').value                 # '' = the input cloud's frame_id
        self.p_verbose = bool(P('verbose', False).value)
        self.p_markers_on = bool(P('publish_markers', True).value)
        # DDS reader history (reliable KEEP_LAST): the executor only queues messages, so this holds
        # frames only while DDS delivers faster than the callback returns.
        self.p_depth = int(P('qos_depth', 100).value)
        # Frames waiting in each of the node's two stages (decode, engine), newest kept: when the
        # engine falls behind the sensor, older frames are dropped (counted, logged) instead of
        # queueing -- latency stays bounded (engine time + at most max_queue frames of waiting).
        # Safe since metro_version 7: the odometry takes the header stamp and bridges the skipped
        # frames (dt / 0.1 s of travel). Before, a skipped frame made the odometry unlock and the
        # node kept a 100-frame queue instead (latency grew without bound under load).
        self.p_max_queue = int(P('max_queue', 1).value)
        # ... except while the odometry is not locked (start, or a lock lost): then every frame is kept
        # (up to warmup_queue = 2 s). The warm-up lock needs consecutive frames (across a gap the lining
        # aliases make its multi-start ambiguous: every 2nd frame dropped from the start locked only at
        # the 30-frame timeout) and an unlocked frame after a gap of n frames registers from ~4n starts
        # instead of 8 (measured in Docker on doubleT_obstacle: frames dropped during the warm-up made the
        # lock frame a 1.0 s registration and the node processed 4-7 of 25 frames).
        self.p_warmup_queue = int(P('warmup_queue', 20).value)
        # ... and a stage never drops so many frames that the frames it processes are more than max_skip
        # seconds apart: a backlog (the warm-up's, a slow frame's) is worked off by taking one frame per
        # max_skip until caught up, not by one jump -- the locked odometry bridges 5 frames cheaply
        # (tracking from the predicted travel; beyond 1 s it would restart its warm-up).
        self.p_max_skip = float(P('max_skip', 0.5).value)
        # hand the header stamp to the engine (metro_version >= 7); false = consecutive frames assumed
        self.p_stamps = bool(P('use_stamps', True).value)
        # organisers' ring-less clouds: rebuild the grid in Rust (metro_version >= 7), else in Python
        self.p_rust_regrid = bool(P('rust_regrid', True).value)
        # reliable: a best-effort subscription loses multi-megabyte clouds (measured: about 60 % of the
        # organisers' frames at 10 Hz), and every lost frame hurts the odometry. The Hesai ROS 2 driver
        # and `ros2 bag play` of the dataset publish reliable; for a best-effort publisher set false
        # (an incompatible-QoS warning is logged).
        self.p_reliable = bool(P('qos_reliable', True).value)
        self.p_record = P('record_path', '').value             # JSONL per frame (offline checks)
        self.p_lib = P('library', '').value or None
        self.p_detections = bool(P('publish_detections', False).value)

        self.pub_alerts = self.create_publisher(String, self.p_alerts, 10)
        self.pub_obst = self.create_publisher(Bool, self.p_obst, 10)
        self.pub_dist = self.create_publisher(Float32, self.p_dist, 10)
        self.pub_status = self.create_publisher(String, self.p_status, 10)
        self.pub_markers = self.create_publisher(MarkerArray, self.p_markers, 10) if self.p_markers_on else None
        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.rec = open(self.p_record, 'a', buffering=1) if self.p_record else None

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=max(1, self.p_depth),
                         reliability=ReliabilityPolicy.RELIABLE if self.p_reliable else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.streams = {}
        self.stopping = False
        for t in self.p_inputs:
            st = Stream(t, self.p_lib, self.p_rust_regrid)
            self.streams[t] = st
            st.thread = threading.Thread(target=self.worker, args=(st,), name=f'metro_engine{len(self.streams)}', daemon=True)
            st.thread.start()
            st.pthread = threading.Thread(target=self.prep, args=(st,), name=f'metro_prep{len(self.streams)}', daemon=True)
            st.pthread.start()
            kw = {}
            if SubscriptionEventCallbacks is not None:
                kw['event_callbacks'] = SubscriptionEventCallbacks(
                    incompatible_qos=lambda ev, t=t: self.get_logger().warn(
                        f'{t}: the publisher QoS is incompatible with this subscription '
                        f'(qos_reliable={self.p_reliable}); for a best-effort publisher set qos_reliable:=false'))
            self.create_subscription(PointCloud2, t, lambda m, s=st: self.on_cloud(s, m), qos, **kw)
        eng = next(iter(self.streams.values())).engine
        self.get_logger().info(
            f'metro_detector: engine metro_version {eng.version}, inputs {self.p_inputs}, alerts -> {self.p_alerts}, '
            f'qos depth {self.p_depth} {"reliable" if self.p_reliable else "best_effort"}, max_queue {self.p_max_queue} '
            f'(warm-up {self.p_warmup_queue}), '
            f'stamps {"on" if self.p_stamps and eng.version >= 7 else "off"}, '
            f'regrid {"rust" if next(iter(self.streams.values())).regridder is not None else "python"}'
            + (f', recording {self.p_record}' if self.rec else ''))
        self.create_timer(1.0, self.on_diag)

    # ---------------------------------------------------------------- per frame
    # Three stages per stream: the subscription callback only queues the message (the executor
    # stays responsive to DDS); the stream's prep thread decodes (and regrids) it; the engine thread
    # runs the engine (the ctypes call releases the GIL) and publishes. Decoding frame k+1 thus
    # overlaps the engine on frame k. Each stage takes the newest frames: at most max_queue wait (warmup_queue
    # while the odometry is not locked), older ones are dropped within the max_skip rule (take()); drops
    # are counted in the status and logged once per second.
    HARD_CAP = 30                                       # frames per stage whatever happens (memory)

    def on_cloud(self, st, msg):
        t0 = time.perf_counter()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with st.pcv:
            if len(st.raw) >= self.HARD_CAP:
                st.raw.popleft()
                st.dropped += 1
            st.raw.append((msg, stamp, t0))
            st.pcv.notify()

    def take(self, st, q, last):
        """the next item of a stage's queue (item[1] = header stamp): older items are dropped while more than
        bound() wait, as long as the item after the dropped one is within max_skip of the last one taken"""
        b = self.bound(st)
        while len(q) > b:
            if last is not None and q[1][1] - last > self.p_max_skip:
                break
            q.popleft()
            st.dropped += 1
        return q.popleft()

    def prep(self, st):
        while True:
            with st.pcv:
                while not st.raw and not self.stopping:
                    st.pcv.wait(0.5)
                if self.stopping:
                    return
                msg, stamp, t0 = self.take(st, st.raw, st.last_prep)
                st.last_prep = stamp
            try:
                a = msg_to_array(msg)
                frame, regridded, _ = engine_frame(a, stamp=stamp, regridder=st.regridder)
                n = int(len(a))
                del a
            except Exception as e:
                self.get_logger().error(f'[{st.topic}] cannot decode a cloud: {e!r}')
                continue
            t1 = time.perf_counter()
            with st.cv:
                if len(st.queue) >= self.HARD_CAP:
                    st.queue.popleft()
                    st.dropped += 1
                st.queue.append((msg.header, stamp, frame, regridded, n, t0, t1))
                st.cv.notify()

    def bound(self, st):
        """frames that may wait in one stage: max_queue once the odometry is locked, warmup_queue before"""
        return max(1, self.p_max_queue if st.locked else max(self.p_max_queue, self.p_warmup_queue))

    def worker(self, st):
        while True:
            with st.cv:
                while not st.queue and not self.stopping:
                    st.cv.wait(0.5)
                if self.stopping:
                    return
                item = self.take(st, st.queue, st.last_eng)
                st.last_eng = item[1]
            try:
                self.process(st, *item)
            except Exception as e:                     # never let one frame kill the stream
                self.get_logger().error(f'[{st.topic}] frame failed: {e!r}')

    def process(self, st, header, stamp, frame, regridded, n_points, t0, t1):
        t1b = time.perf_counter()
        r = st.engine.step(frame, verbose=self.p_markers_on, stamp=stamp if self.p_stamps else None)
        t2 = time.perf_counter()
        k = st.k
        st.k += 1
        odo = r.get('odo') or {}
        st.locked = bool(odo.get('locked', False))
        gap = odo.get('frames') or 1.0
        st.gaps += gap != 1.0
        alerts = []
        for al in r['alerts']:
            new = al['id'] not in st.seen
            st.seen.add(al['id'])
            alerts.append(dict(al, new=new))
        alerts.sort(key=lambda q: q['d'] if q['d'] is not None else math.inf)
        nearest = alerts[0] if alerts else None
        frame_id = self.p_frame or header.frame_id
        g = r.get('geom') or {}
        out = {
            'stamp': stamp, 'frame_id': frame_id, 'topic': st.topic, 'seq': k,
            'obstacle': bool(alerts),
            'nearest_distance': _r(nearest['d']) if nearest else None,
            'alerts': [{'id': q['id'], 'distance': _r(q['d']), 'lateral': _r(q['lateral']), 'height': _r(q['height']),
                        'n_points': q['n'], 'hits': q['hits'], 'first_alert_distance': _r(q['first_alert_d']),
                        'new': q['new']} for q in alerts],
            'odometry': {'locked': st.locked, 'ds': _r(r.get('ds'), 4), 's': _r(r.get('s'), 3),
                         'event': odo.get('event'), 'frames': _r(gap, 2)},
            'geometry': {'ok': bool(r.get('geom_ok')), 'reach': _r(g.get('reach'), 1),
                         'los_horizon': _r(g.get('los_horizon'), 1), 'degraded': g.get('degraded')},
        }
        if self.p_detections:
            out['detections'] = r.get('detections', [])
        # publish
        self.pub_alerts.publish(String(data=json.dumps(out)))
        self.pub_obst.publish(Bool(data=bool(alerts)))
        self.pub_dist.publish(Float32(data=float(nearest['d']) if nearest else -1.0))
        if self.pub_markers is not None:
            self.pub_markers.publish(self.markers(header, frame_id, alerts, g))
        t3 = time.perf_counter()
        lat = {'decode': round((t1 - t0) * 1e3, 2), 'queue': round((t1b - t1) * 1e3, 2), 'engine': round((t2 - t1b) * 1e3, 2),
               'odometry': _r((odo.get('ms') or {}).get('total'), 2), 'publish': round((t3 - t2) * 1e3, 2),
               'total': round((t3 - t0) * 1e3, 2)}
        st.t_total.append(lat['total'])
        st.t_arrival.append(time.monotonic())
        st.alert_frames += bool(alerts)
        st.last = dict(seq=k, stamp=stamp, locked=st.locked, latency_ms=lat, regrid=regridded, n_points=n_points)
        status = {'topic': st.topic, 'seq': k, 'stamp': stamp, 'odometry_locked': st.locked,
                  'odometry_event': odo.get('event'), 'frames_unlocked': odo.get('frames_unlocked'),
                  'latency_ms': lat, 'latency_p50_ms': _r(np.median(st.t_total), 2),
                  'input_hz': _r(self._rate(st), 2), 'regrid': regridded, 'n_points': n_points,
                  'alert_frames': st.alert_frames, 'objects_alerted': len(st.seen), 'dropped_frames': st.dropped,
                  'odometry_frames': _r(gap, 2), 'gaps_bridged': st.gaps}
        self.pub_status.publish(String(data=json.dumps(status)))
        if self.rec is not None:
            self.rec.write(json.dumps({'topic': st.topic, 'k': k, 'stamp': stamp, 'n_points': n_points, 'regrid': regridded,
                                       'alerts': r['alerts'], 's': r.get('s'), 'ds': r.get('ds'), 'locked': st.locked,
                                       'latency_ms': lat, 'dropped': st.dropped, 'queued': len(st.queue),
                                       'frames': gap, 'event': odo.get('event')}) + '\n')
        if self.p_verbose or alerts and any(q['new'] for q in alerts):
            nz = lambda v: math.nan if v is None else v
            msg_ = ', '.join(f"#{q['id']} {nz(q['d']):.1f} m (lat {nz(q['lateral']):+.2f}, h {nz(q['height']):.2f})"
                             for q in alerts) or 'clear'
            self.get_logger().info(f"[{st.topic} {k}] {'LOCK ' if st.locked else 'warm-up '}{msg_}  "
                                   f"frame {lat['total']:.0f} ms (engine {lat['engine']:.0f})")

    @staticmethod
    def _rate(st):
        if len(st.t_arrival) < 2:
            return None
        dt = st.t_arrival[-1] - st.t_arrival[0]
        return (len(st.t_arrival) - 1) / dt if dt > 0 else None

    # ---------------------------------------------------------------- RViz
    def markers(self, header, frame_id, alerts, g):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = frame_id
        clear.header.stamp = header.stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        D, X, FD, FZ = (np.asarray([np.nan if v is None else v for v in (g.get(k) or [])], float) for k in ('D', 'X', 'FD', 'FZ'))
        okc = np.isfinite(D) & np.isfinite(X) if len(D) == len(X) else np.zeros(0, bool)
        okz = np.isfinite(FD) & np.isfinite(FZ) if len(FD) == len(FZ) else np.zeros(0, bool)
        cx = lambda d: float(np.interp(d, D[okc], X[okc])) if okc.sum() >= 2 else 0.0
        cz = lambda d: float(np.interp(d, FD[okz], FZ[okz])) if okz.sum() >= 2 else 0.0

        def mk(ns, i, typ):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = header.stamp
            m.ns = ns; m.id = i; m.type = typ; m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m
        for i, q in enumerate(alerts):
            d, u, h = q['d'], q['lateral'], q['height']
            if d is None or u is None or h is None:
                continue
            x, z0 = cx(d) + u, cz(d)
            hh = max(abs(h), 0.15)
            b = mk('obstacles', i, Marker.CUBE)
            # sensor frame: forward = -y, lateral = x, up = z
            b.pose.position.x = x; b.pose.position.y = -d; b.pose.position.z = z0 + (h / 2 if h > 0 else 0.0)
            b.scale.x = 0.6; b.scale.y = 0.6; b.scale.z = hh
            b.color.r = 1.0; b.color.g = 0.1; b.color.b = 0.1; b.color.a = 0.8
            ma.markers.append(b)
            t = mk('labels', i, Marker.TEXT_VIEW_FACING)
            t.pose.position.x = x; t.pose.position.y = -d; t.pose.position.z = z0 + hh + 0.8
            t.scale.z = 0.8
            t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
            t.text = f"#{q['id']} {d:.1f} m"
            ma.markers.append(t)
        if okc.sum() >= 2:
            c = mk('centreline', 0, Marker.LINE_STRIP)
            c.scale.x = 0.08
            c.color.g = 0.9; c.color.b = 0.3; c.color.a = 0.9
            for d, x in zip(D[okc][::2], X[okc][::2]):
                c.points.append(Point(x=float(x), y=float(-d), z=cz(d)))
            ma.markers.append(c)
        return ma

    # ---------------------------------------------------------------- diagnostics
    def on_diag(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for st in self.streams.values():
            s = DiagnosticStatus()
            s.name = f'metro_detector: {st.topic}'
            s.hardware_id = 'hesai_ot128'
            last = st.last
            if last is None:
                s.level = DiagnosticStatus.WARN; s.message = 'no frames yet'
            elif not last['locked']:
                s.level = DiagnosticStatus.WARN; s.message = 'odometry warming up (alerts suppressed)'
            else:
                s.level = DiagnosticStatus.OK; s.message = 'running'
            p50 = float(np.median(st.t_total)) if st.t_total else float('nan')
            p95 = float(np.percentile(st.t_total, 95)) if st.t_total else float('nan')
            s.values = [KeyValue(key='frames', value=str(st.k)),
                        KeyValue(key='odometry_locked', value=str(st.locked)),
                        KeyValue(key='frame_time_p50_ms', value=f'{p50:.1f}'),
                        KeyValue(key='frame_time_p95_ms', value=f'{p95:.1f}'),
                        KeyValue(key='input_hz', value=str(_r(self._rate(st), 2))),
                        KeyValue(key='objects_alerted', value=str(len(st.seen))),
                        KeyValue(key='dropped_frames', value=str(st.dropped)),
                        KeyValue(key='gaps_bridged', value=str(st.gaps))]
            if st.dropped > st.dropped_logged:
                self.get_logger().warn(f'[{st.topic}] dropped {st.dropped - st.dropped_logged} frame(s) in the last second '
                                       f'(total {st.dropped}; the engine is slower than the input, newest frames kept)')
                st.dropped_logged = st.dropped
            if st.dropped and s.level == DiagnosticStatus.OK:
                s.level = DiagnosticStatus.WARN; s.message = 'running, frames dropped (processing slower than input)'
            arr.status.append(s)
        self.pub_diag.publish(arr)

    def destroy_node(self):
        self.stopping = True
        for st in self.streams.values():
            with st.cv:
                st.cv.notify_all()
            with st.pcv:
                st.pcv.notify_all()
        for st in self.streams.values():
            st.thread.join(timeout=5.0)
            st.pthread.join(timeout=5.0)
        if self.rec is not None:
            self.rec.close()
        for st in self.streams.values():
            if not st.thread.is_alive():
                st.engine.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetroDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
