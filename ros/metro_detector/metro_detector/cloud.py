"""PointCloud2 -> the engine's input layout.

Decoding follows the PointCloud2 decoder exactly: a numpy structured
dtype built from the message's own field list (name, offset, datatype) with itemsize = point_step,
over the raw data buffer -- no per-point Python work, any field order or padding.

The engine expects the Hesai OT128 dual-return grid as the driver publishes it: column-major,
slot = column * 128 + ring (each firing twice: column 2k = last echo, 2k+1 = strongest), empty
returns as x = y = z = 0. A cloud already in that layout (ring field present, ring == slot % 128)
is passed through unchanged. Anything else -- no ring field (the organisers' synthetic bag),
a point count that is not a multiple of 128, NaN-filtered or reordered clouds -- is rebuilt onto
the grid from beam geometry by regrid() (the sensor-grid rebuild).
"""
import os

import numpy as np

DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
STD = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'), ('ring', '<u2'), ('timestamp', '<f8')])
_RING_TILE = {}


def to_array(fields, point_step, data, is_bigendian=False):
    """fields: [(name, offset, datatype, count)] -> structured array over data."""
    fields = [f for f in fields if f[2] in DT]
    bo = '>' if is_bigendian else '<'
    dt = np.dtype({'names': [f[0] for f in fields],
                   'formats': [bo + DT[f[2]] for f in fields],
                   'offsets': [f[1] for f in fields],
                   'itemsize': point_step})
    n = len(data) // point_step
    return np.frombuffer(data, dtype=dt, count=n)


def msg_to_array(msg):
    """sensor_msgs/msg/PointCloud2 -> structured array (no copy of the point data)."""
    fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    n = msg.width * msg.height
    data = memoryview(msg.data)[:n * msg.point_step] if n else memoryview(msg.data)
    return to_array(fields, msg.point_step, data, bool(msg.is_bigendian))


def _ring_tile(n):
    t = _RING_TILE.get(n)
    if t is None:
        t = np.tile(np.arange(128, dtype=np.uint16), n // 128)
        _RING_TILE[n] = t
    return t


def in_grid_layout(a):
    """True when a is already the driver's column-major 128-ring grid."""
    names = a.dtype.names
    if 'ring' not in names or len(a) == 0 or len(a) % 128:
        return False
    return bool(np.array_equal(a['ring'], _ring_tile(len(a))))


def engine_frame(a, stamp=0.0, regridder=None):
    """structured array -> (dict of contiguous engine arrays, regridded: bool, stats or None).
    regridder: engine.Regridder (the Rust regrid, byte-identical) or None (the Python regrid_cols)."""
    names = a.dtype.names
    if in_grid_layout(a):
        c = lambda f, t: np.ascontiguousarray(a[f], dtype=t)
        x, y, z = c('x', np.float32), c('y', np.float32), c('z', np.float32)
        bad = ~(np.isfinite(x) & np.isfinite(y) & np.isfinite(z))
        if bad.any():                                  # the dataset uses zeros for no return; NaN drivers too
            x = x.copy(); y = y.copy(); z = z.copy()
            x[bad] = 0; y[bad] = 0; z[bad] = 0
        it = c('intensity', np.float32) if 'intensity' in names else np.zeros(len(a), np.float32)
        ts = c('timestamp', np.float64) if 'timestamp' in names else None
        return dict(x=x, y=y, z=z, intensity=it, ring=c('ring', np.uint16), timestamp=ts), False, None
    x, y, z = a['x'], a['y'], a['z']
    it = a['intensity'] if 'intensity' in names else np.zeros(len(a), np.float32)
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    if not ok.all():
        x, y, z, it = x[ok], y[ok], z[ok], it[ok]
    cols, st = (regridder or regrid_cols)(x, y, z, it)
    return cols, True, st


def make_regridder(lib_path=None):
    """the Rust regrid over this module's sensor table, or None when the library predates it (Python fallback)"""
    from .engine import Regridder, load
    if int(load(lib_path).metro_version()) < 7:
        return None
    T = _table()
    if 'tv' not in T:                                   # as regrid_cols caches them
        T['tv'] = T['azs'][T['azs'] < 1e5]
        T['ph_tv'] = _ph(T['tv'])
    return Regridder(T, T['ph_tv'], lib_path)


def _ph(a, step=0.1):
    # regrid_cols' ph() (phase of a set of azimuths on the 0.1 deg firing grid)
    return np.degrees(np.angle(np.mean(np.exp(1j * np.radians(np.mod(a, step) / step * 360.0))))) / 360.0 * step


# ---- regrid: the sensor-grid rebuild (organisers' synthetic bags) --------------------------------------------

TABLE = os.environ.get('METRO_SENSOR_TABLE') or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sensor_hesai128.npz')
_T = None


def _table():
    global _T
    if _T is None:
        z = np.load(TABLE)
        el = z['elevation_deg'].astype(float)
        dirs = z['dirs'].astype(float)                     # (128,), (128, W, 3)
        # azimuth in the SAME convention as the points (atan2(y, x)), from the measured unit directions
        az = np.degrees(np.arctan2(dirs[..., 1], dirs[..., 0]))
        az[~z['valid']] = 1e6                               # never-valid slots sort to the end, never matched
        W = az.shape[1]
        npair = W // 2
        azp = az[:, 1::2]
        order = np.argsort(azp, axis=1)
        azs = np.take_along_axis(azp, order, axis=1)
        eo = np.argsort(el)
        _T = dict(el=el, eo=eo, els=el[eo], azs=azs, order=order, W=W, npair=npair, valid=z['valid'])
    return _T


_OUT0 = {}


def regrid(a4, stamp=0.0):
    """structured x, y, z, intensity points in any order -> (the standard 128 x W dual-return grid
    as a structured array with ring / timestamp, stats) -- the sensor-grid rebuild interface."""
    c, stats = regrid_cols(a4['x'], a4['y'], a4['z'], a4['intensity'])
    out = np.zeros(len(c['x']), STD)
    for f in ('x', 'y', 'z', 'intensity', 'ring'):
        out[f] = c[f]
    out['timestamp'] = stamp
    stats['n_in'] = int(len(a4))
    return out, stats


def regrid_cols(xs, ys, zs, its):
    """points in any order -> (dict of the grid's columns x, y, z, intensity float32, ring uint16,
    timestamp None; stats). The engine reads the columns directly.

    Output identical to the reference sensor-grid rebuild,
    vectorised: one stable sort by channel instead of a mask per channel, a stable argsort instead
    of lexsort, and the "more than two points per firing" case with segment reductions instead of a
    Python loop (about 3x faster on the organisers' bag).
    """
    T = _table()
    W, npair = T['W'], T['npair']
    x = np.asarray(xs).astype(float); y = np.asarray(ys).astype(float); z = np.asarray(zs).astype(float)
    v = (x != 0) | (y != 0) | (z != 0)
    xv, yv, zv, iv = x[v], y[v], z[v], np.asarray(its)[v]
    rng = np.sqrt(xv**2 + yv**2 + zv**2)
    elev = np.degrees(np.arcsin(np.clip(zv / np.maximum(rng, 1e-9), -1, 1)))
    azim = np.degrees(np.arctan2(yv, xv))
    # nearest channel by elevation
    j = np.clip(np.searchsorted(T['els'], elev), 1, len(T['els']) - 1)
    jl = np.where(np.abs(elev - T['els'][j - 1]) < np.abs(elev - T['els'][j]), j - 1, j)
    ring = T['eo'][jl]
    # encoder phase: per-frame phase difference against the table's 0.1 deg firing grid shifts the table
    step = 0.1
    ph = lambda a: np.degrees(np.angle(np.mean(np.exp(1j * np.radians(np.mod(a, step) / step * 360.0))))) / 360.0 * step
    if 'tv' not in T:
        T['tv'] = T['azs'][T['azs'] < 1e5]
        T['ph_tv'] = ph(T['tv'])
    shift = (ph(azim) - T['ph_tv'] + step / 2) % step - step / 2 if len(azim) else 0.0
    # nearest firing pair by azimuth within that channel (points of one channel in index order)
    pair = np.zeros(len(azim), int)
    by_ring = np.argsort(ring, kind='stable')
    bounds = np.searchsorted(ring[by_ring], np.arange(129))
    for r in range(128):
        q = by_ring[bounds[r]:bounds[r + 1]]
        if len(q) == 0:
            continue
        a_ = T['azs'][r] + shift
        aq = azim[q]
        k = np.clip(np.searchsorted(a_, aq), 1, npair - 1)
        kl = np.where(np.abs(aq - a_[k - 1]) < np.abs(aq - a_[k]), k - 1, k)
        pair[q] = T['order'][r, kl]
    ring_tile = _OUT0.get(W)
    if ring_tile is None:
        ring_tile = _OUT0[W] = np.tile(np.arange(128, dtype=np.uint16), W)
    out = {f: np.zeros(W * 128, np.float32) for f in ('x', 'y', 'z', 'intensity')}
    slot = pair * 128 + ring
    o = np.argsort(slot, kind='stable')                   # == lexsort((arange, slot))
    s_sorted = slot[o]
    st = np.flatnonzero(np.r_[True, s_sorted[1:] != s_sorted[:-1]]) if len(o) else np.zeros(0, int)
    en = np.r_[st[1:], len(o)]
    cnt = en - st

    def put(col_twin, pidx, sl):
        c = (sl // 128) * 2 + col_twin
        r = sl % 128
        fi = c * 128 + r
        out['x'][fi] = xv[pidx]; out['y'][fi] = yv[pidx]; out['z'][fi] = zv[pidx]; out['intensity'][fi] = iv[pidx]
    one = cnt == 1
    put(0, o[st[one]], s_sorted[st[one]]); put(1, o[st[one]], s_sorted[st[one]])
    two = cnt == 2
    put(0, o[st[two]], s_sorted[st[two]]); put(1, o[st[two] + 1], s_sorted[st[two]])
    many = np.flatnonzero(cnt > 2)
    if len(many):
        # per firing with > 2 points: the nearest is the strongest twin, the farthest the last
        # (first occurrence in stream order on ties, as argmin / argmax)
        gid = np.repeat(np.arange(len(st)), cnt)
        rv = rng[o]
        gmin = np.minimum.reduceat(rv, st)
        gmax = np.maximum.reduceat(rv, st)
        pmin = np.flatnonzero(rv == gmin[gid]); pmin = pmin[np.searchsorted(gid[pmin], many)]
        pmax = np.flatnonzero(rv == gmax[gid]); pmax = pmax[np.searchsorted(gid[pmax], many)]
        put(0, o[pmax], s_sorted[st[many]]); put(1, o[pmin], s_sorted[st[many]])
    out['ring'] = ring_tile.copy()
    out['timestamp'] = None
    stats = dict(n_in=int(len(x)), n_valid=int(v.sum()), slots_1=int(one.sum()), slots_2=int(two.sum()),
                 slots_many=int(len(many)), az_shift=float(shift))
    return out, stats
