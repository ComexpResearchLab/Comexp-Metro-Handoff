"""Machine fingerprint, computed identically at build time and run time.

This exact file is used by activate.py (build time, inside `docker build`) and by the
node's loader (run time, inside `docker run`). Both must derive the same parts from the
same hardware or the machine binding is meaningless, so there is one implementation here
and no second copy.

Unique parts (bind the engine to one machine):
  uuid   /sys/class/dmi/id/product_uuid
  board  /sys/class/dmi/id/board_serial
  nvme   sorted, comma-joined /sys/class/nvme/nvme*/serial

Context parts (LOGGED ONLY, never fed into a key): these are identical on every stand of
the same model, so they carry no binding value -- they only help a human tell stands apart
in the audit log.
  cpu             "model name" from /proc/cpuinfo
  sys_vendor      /sys/class/dmi/id/sys_vendor
  product_name    /sys/class/dmi/id/product_name

Normalisation: trim whitespace; treat known BIOS placeholders as ABSENT so we never bind to
a value every ASUS/whitebox board ships with (verified on the real stand: product_serial was
"System Serial Number", a placeholder, while product_uuid and board_serial were real).
"""
import glob
import hashlib
import os

# Values a BIOS ships unfilled. Compared case-insensitively after trimming. An all-zero uuid
# and the empty string are also placeholders (handled in _clean).
_PLACEHOLDERS = {
    'system serial number',
    'default string',
    'to be filled by o.e.m.',
    'to be filled by o.e.m',
    'none',
    'not specified',
    'not available',
    '0',
}

# Order matters: the key derivation joins parts sorted by name, and this is that sort order.
UNIQUE_NAMES = ('board', 'nvme', 'uuid')


def _read(path):
    try:
        with open(path, 'r', errors='replace') as f:
            return f.read()
    except OSError:
        return None


def _clean(value):
    """Trim; return None for any placeholder / empty / all-zero value."""
    if value is None:
        return None
    v = value.strip()
    if not v:
        return None
    low = v.lower()
    if low in _PLACEHOLDERS:
        return None
    # all-zero uuid (e.g. 00000000-0000-0000-0000-000000000000)
    if set(low) <= {'0', '-'} and '0' in low:
        return None
    return v


def _read_unique():
    """The three unique parts, each cleaned; absent ones are omitted."""
    parts = {}
    uuid = _clean(_read('/sys/class/dmi/id/product_uuid'))
    if uuid:
        parts['uuid'] = uuid
    board = _clean(_read('/sys/class/dmi/id/board_serial'))
    if board:
        parts['board'] = board
    serials = []
    for path in sorted(glob.glob('/sys/class/nvme/nvme*/serial')):
        s = _clean(_read(path))
        if s:
            serials.append(s)
    # sort the cleaned serials so add/remove of an unrelated disk keeps the order stable
    serials = sorted(set(serials))
    if serials:
        parts['nvme'] = ','.join(serials)
    return parts


def _cpu_model():
    text = _read('/proc/cpuinfo') or ''
    for line in text.splitlines():
        if line.lower().startswith('model name'):
            _, _, val = line.partition(':')
            return _clean(val)
    return None


def _read_context():
    ctx = {}
    cpu = _cpu_model()
    if cpu:
        ctx['cpu'] = cpu
    for name, path in (('sys_vendor', '/sys/class/dmi/id/sys_vendor'),
                       ('product_name', '/sys/class/dmi/id/product_name')):
        v = _clean(_read(path))
        if v:
            ctx[name] = v
    return ctx


def collect():
    """Return {'unique': {name: value}, 'context': {name: value}}.

    'unique' holds only the present, non-placeholder unique parts (0..3 entries).
    """
    return {'unique': _read_unique(), 'context': _read_context()}


def machine_id(unique):
    """Stable id of a machine: SHA256 over the sorted unique parts, hex.

    Same formula on server and client so a stand keeps one id across activations. Empty
    unique -> a fixed sentinel so an unidentifiable machine is still logged distinctly.
    """
    if not unique:
        return 'nomachine'
    joined = '\n'.join(f'{k}={unique[k]}' for k in sorted(unique))
    return hashlib.sha256(joined.encode()).hexdigest()


def _override_from_env():
    """Test hook: METRO_FP_OVERRIDE='uuid=..;board=..;nvme=..' replaces the unique parts.

    Lets us prove the machine binding refuses on a different fingerprint without needing a
    second physical machine. Never set in a delivery; the loader logs loudly when it is set.
    """
    raw = os.environ.get('METRO_FP_OVERRIDE')
    if not raw:
        return None
    parts = {}
    for item in raw.split(';'):
        item = item.strip()
        if not item or '=' not in item:
            continue
        k, _, v = item.partition('=')
        k = k.strip()
        v = v.strip()
        if k in UNIQUE_NAMES and v:
            parts[k] = v
    return parts


def collect_effective():
    """collect(), but honour METRO_FP_OVERRIDE for the unique parts (test mode).

    Returns (fp_dict, overridden: bool).
    """
    fp = collect()
    ov = _override_from_env()
    if ov is not None:
        fp['unique'] = ov
        return fp, True
    return fp, False
