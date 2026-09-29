"""ctypes binding of the detection engine library (libmetro_core.so, C ABI).

One Engine per point-cloud stream. Every frame goes through metro_tracker_step2 with ds = NaN:
the tracker then measures its own odometry (built-in point-to-plane ICP, warm-up speed lock) and
integrates the full sensor pose -- no external odometry is needed. The step result is the JSON
the library returns (alerts, detections, s, ds, geom, odo, x0p).

metro_version >= 7: step(frame, stamp=t) hands the frame's sensor stamp (the PointCloud2 header
stamp) to the engine first (metro_tracker_set_stamp): its odometry then bridges lost or dropped
frames (dt / 0.1 s frames of travel) instead of unlocking. And Regridder: the organisers' ring-less
clouds rebuilt onto the sensor grid by the engine (metro_regrid).

Library path: the constructor argument, else $METRO_LIB, else /opt/metro/lib/libmetro_core.so.
"""
import ctypes
import json
import math
import os

import numpy as np

DEFAULT_LIB = '/opt/metro/lib/libmetro_core.so'
MIN_VERSION = 5          # metro_tracker_step2 (v46 pose step)

_libs = {}


def _resolve_activated():
    """Delivery mode: decrypt the machine-bound engine into an in-memory fd and return its
    /proc/self/fd path. Import is lazy so a dev checkout (METRO_LIB set) never needs the
    activation package or `cryptography`. Any failure aborts the process loudly -- there is no
    fallback engine, because this is a safety system."""
    try:
        from activation.loader import resolve_engine, ActivationError
    except ImportError as e:
        raise RuntimeError(
            'metro_detector: no engine is available. Set METRO_LIB to a licensed engine, or '
            f'build the image with the activation step (could not import the loader: {e}).')
    try:
        return resolve_engine()
    except ActivationError as e:
        # Loud, actionable message already formatted by the loader; do not wrap it.
        import sys
        sys.stderr.write(str(e) + '\n')
        raise SystemExit(3)


def load(path=None):
    # No explicit path and no METRO_LIB override -> delivery mode: unlock the licensed engine.
    if path is None and not os.environ.get('METRO_LIB'):
        path = _resolve_activated()
    path = path or os.environ.get('METRO_LIB') or DEFAULT_LIB
    if path in _libs:
        return _libs[path]
    L = ctypes.CDLL(path)
    L.metro_version.restype = ctypes.c_int32
    ver = L.metro_version()
    if ver < MIN_VERSION:
        raise RuntimeError(f'{path}: metro_version {ver} < {MIN_VERSION}')
    # Licensed engine builds expose metro_license_check: verify the machine binding once, up front,
    # so an unlicensed machine gets a clean non-zero exit here rather than a SIGABRT later from the
    # hard gate in metro_tracker_new. Dev builds do not export it (or return 0).
    if hasattr(L, 'metro_license_check'):
        L.metro_license_check.restype = ctypes.c_int32
        if int(L.metro_license_check()) != 0:
            raise RuntimeError('metro engine licence check failed: this engine is licensed to a '
                               'different machine (see the message above). Refusing to run.')
    f32p = ctypes.POINTER(ctypes.c_float)
    f64p = ctypes.POINTER(ctypes.c_double)
    L.metro_tracker_new.restype = ctypes.c_void_p
    L.metro_tracker_free.argtypes = [ctypes.c_void_p]
    L.metro_str_free.argtypes = [ctypes.c_void_p]
    L.metro_tracker_step2.argtypes = [ctypes.c_void_p, ctypes.c_int64, f32p, f32p, f32p, f32p,
                                      ctypes.POINTER(ctypes.c_uint16), f64p, ctypes.c_double, f64p,
                                      ctypes.POINTER(ctypes.c_uint8), ctypes.c_int32]
    L.metro_tracker_step2.restype = ctypes.c_void_p
    if ver >= 7:
        L.metro_tracker_set_stamp.argtypes = [ctypes.c_void_p, ctypes.c_double]
        L.metro_tracker_set_stamp.restype = None
        i64p = ctypes.POINTER(ctypes.c_int64)
        L.metro_regrid_new.argtypes = [ctypes.c_int64, f64p, i64p, f64p, i64p, ctypes.c_double]
        L.metro_regrid_new.restype = ctypes.c_void_p
        L.metro_regrid_free.argtypes = [ctypes.c_void_p]
        L.metro_regrid.argtypes = [ctypes.c_void_p, ctypes.c_int64, f32p, f32p, f32p, f32p, f32p, f32p, f32p, f32p, f64p]
        L.metro_regrid.restype = ctypes.c_int32
    _libs[path] = L
    return L


class Engine:
    """One tracker (one stream). step(frame) -> dict (the library's step JSON)."""

    def __init__(self, lib_path=None):
        self.L = load(lib_path)
        self.version = int(self.L.metro_version())
        self._h = ctypes.c_void_p(self.L.metro_tracker_new())
        self.frames = 0

    def close(self):
        if self._h:
            self.L.metro_tracker_free(self._h)
            self._h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def step(self, frame, verbose=False, stamp=None):
        """frame: dict of contiguous arrays x, y, z, intensity (float32), ring (uint16),
        timestamp (float64 or None); n = number of slots (a multiple of 128, column-major).
        stamp: the frame's sensor stamp in seconds (header stamp), or None (consecutive frames assumed)."""
        def p(a, t):
            return a.ctypes.data_as(ctypes.POINTER(t)) if a is not None else None
        x, y, z, it, ring = frame['x'], frame['y'], frame['z'], frame['intensity'], frame['ring']
        ts = frame.get('timestamp')
        n = len(x)
        for a, t in ((x, np.float32), (y, np.float32), (z, np.float32), (it, np.float32), (ring, np.uint16)):
            assert a.dtype == t and a.flags['C_CONTIGUOUS'] and len(a) == n
        if stamp is not None and self.version >= 7:
            self.L.metro_tracker_set_stamp(self._h, float(stamp))
        q = self.L.metro_tracker_step2(self._h, n, p(x, ctypes.c_float), p(y, ctypes.c_float), p(z, ctypes.c_float),
                                       p(it, ctypes.c_float), p(ring, ctypes.c_uint16), p(ts, ctypes.c_double),
                                       math.nan, None, None, 1 if verbose else 0)
        if not q:
            raise RuntimeError('metro_core: step panicked (message on stderr)')
        try:
            r = json.loads(ctypes.string_at(q).decode())
        finally:
            self.L.metro_str_free(q)
        self.frames += 1
        return r


class Regridder:
    """the sensor-grid rebuild in the engine library (metro_version >= 7): points in any order -> the
    128 x W grid columns, as cloud.regrid_cols returns them. table: cloud._table() (the Python
    derivation of the sensor table, so both paths use the same numbers)."""

    def __init__(self, table, ph_tv, lib_path=None):
        self.L = load(lib_path)
        if int(self.L.metro_version()) < 7:
            raise RuntimeError('metro_regrid needs metro_version >= 7')
        T = table
        self.W = int(T['W'])
        self._keep = [np.ascontiguousarray(T['els'], np.float64), np.ascontiguousarray(T['eo'], np.int64),
                      np.ascontiguousarray(T['azs'], np.float64).reshape(-1), np.ascontiguousarray(T['order'], np.int64).reshape(-1)]
        f64p, i64p = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int64)
        e, o, a, r = self._keep
        self._h = ctypes.c_void_p(self.L.metro_regrid_new(self.W, e.ctypes.data_as(f64p), o.ctypes.data_as(i64p),
                                                          a.ctypes.data_as(f64p), r.ctypes.data_as(i64p), float(ph_tv)))
        self._ring = np.tile(np.arange(128, dtype=np.uint16), self.W)

    def close(self):
        if self._h:
            self.L.metro_regrid_free(self._h)
            self._h = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def __call__(self, xs, ys, zs, its):
        """-> (dict x, y, z, intensity float32, ring uint16, timestamp None; stats) as cloud.regrid_cols"""
        f32p = ctypes.POINTER(ctypes.c_float)
        c = lambda v: np.ascontiguousarray(v, dtype=np.float32)
        x, y, z, it = c(xs), c(ys), c(zs), c(its)
        m = self.W * 128
        out = {f: np.empty(m, np.float32) for f in ('x', 'y', 'z', 'intensity')}
        st = np.zeros(6, np.float64)
        P = lambda v, t: v.ctypes.data_as(ctypes.POINTER(t))
        rc = self.L.metro_regrid(self._h, len(x), P(x, ctypes.c_float), P(y, ctypes.c_float), P(z, ctypes.c_float), P(it, ctypes.c_float),
                                 P(out['x'], ctypes.c_float), P(out['y'], ctypes.c_float), P(out['z'], ctypes.c_float),
                                 P(out['intensity'], ctypes.c_float), P(st, ctypes.c_double))
        if rc != 0:
            raise RuntimeError('metro_regrid panicked (message on stderr)')
        out['ring'] = self._ring.copy()
        out['timestamp'] = None
        stats = dict(n_in=int(st[0]), n_valid=int(st[1]), slots_1=int(st[2]), slots_2=int(st[3]), slots_many=int(st[4]), az_shift=float(st[5]))
        return out, stats
