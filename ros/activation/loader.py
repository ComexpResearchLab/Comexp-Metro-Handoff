"""Run-time engine loader (part of the delivered node package).

At `docker run`, before the node touches any lidar data, this:
  1. recomputes the machine fingerprint (fingerprint.py, the same code the build used),
  2. finds the subset key that opens one of the shipped wraps -> the content key K,
  3. decrypts the engine .so with K into an ANONYMOUS in-memory file (memfd_create; a private
     0600 tmpfs file only as a fallback) so the plaintext .so never lands on the image
     filesystem, and
  4. returns a path under /proc/self/fd that ctypes.CDLL can dlopen.

On any failure it raises ActivationError with a loud, actionable message that names the licensed
machine and a contact address. The caller (engine.py / node.py) must let that abort the process
with a non-zero exit code. There is NO degraded mode and NO fallback engine: this is a safety
system, so a machine that cannot prove its licence does not run at all.
"""
import ctypes
import ctypes.util
import json
import os
import sys

from . import fingerprint, wire

DEFAULT_DIR = '/opt/metro/activation'
CONTACT = os.environ.get('METRO_CONTACT', 'obstacle-detector@comexp.net')


class ActivationError(Exception):
    pass


def _load_cryptography():
    try:
        from cryptography.hazmat.primitives.ciphers.aead import ChaCha20Poly1305
        from cryptography.hazmat.primitives.kdf.hkdf import HKDF
        from cryptography.hazmat.primitives import hashes
        return ChaCha20Poly1305, HKDF, hashes
    except Exception as e:  # pragma: no cover - environment problem
        raise ActivationError(
            'the Python `cryptography` package is required to open the licensed engine '
            f'but could not be imported ({e}). Rebuild the image so `pip install cryptography` '
            'runs, or contact ' + CONTACT)


def _subset_key(HKDF, hashes, build_id, subset):
    kdf = HKDF(algorithm=hashes.SHA256(), length=wire.KEY_LEN,
               salt=build_id.encode('utf-8'), info=wire.HKDF_INFO)
    return kdf.derive(wire.subset_ikm(subset))


def _aead_open(ChaCha20Poly1305, key, blob):
    """blob = nonce || ct+tag -> plaintext, or None if it does not authenticate."""
    if len(blob) <= wire.NONCE_LEN:
        return None
    nonce, ct = blob[:wire.NONCE_LEN], blob[wire.NONCE_LEN:]
    try:
        return ChaCha20Poly1305(key).decrypt(nonce, ct, None)
    except Exception:
        return None


def _fatal(message, machine_id, present, licensed_hint=None):
    lines = [
        '',
        '=' * 72,
        'METRO OBSTACLE DETECTOR: ACTIVATION FAILED -- REFUSING TO RUN',
        '=' * 72,
        message,
        '',
        f'This machine id: {machine_id}',
        f'Unique parts present here: {present or "(none readable)"}',
    ]
    if licensed_hint:
        lines.append(f'Engine was licensed for: {licensed_hint}')
    lines += [
        '',
        'This engine is licensed to one machine. It will not run here.',
        f'If this machine is the licensed stand, contact {CONTACT} with the machine id above',
        'and we will re-issue. This is a safety component: it does not run in a degraded mode.',
        '=' * 72,
        '',
    ]
    raise ActivationError('\n'.join(lines))


def resolve_engine(activation_dir=None):
    """Decrypt the licensed engine and return a dlopen-able path (/proc/self/fd/N).

    Keeps the backing fd open for the process lifetime (stashed on the returned function's
    module) so the anonymous file is not reclaimed. Raises ActivationError on any failure.
    """
    activation_dir = activation_dir or os.environ.get('METRO_ACTIVATION_DIR') or DEFAULT_DIR
    manifest_path = os.path.join(activation_dir, 'manifest.json')
    # The encrypted engine ships committed in the delivery repo (COPY'd into the image), not fetched:
    # its git commit fixes the binary at delivery time. Activation only returns the machine-bound key
    # (manifest.json). METRO_ENGINE_ENC overrides the baked path; fall back to the old in-package
    # location so an older, fetch-style package still opens.
    engine_path = os.environ.get('METRO_ENGINE_ENC') or '/opt/metro/engine.enc'
    if not os.path.exists(engine_path):
        engine_path = os.path.join(activation_dir, 'engine.enc')

    fp, overridden = fingerprint.collect_effective()
    unique = fp['unique']
    present = sorted(unique)
    mid = fingerprint.machine_id(unique)
    if overridden:
        sys.stderr.write('metro activation: WARNING METRO_FP_OVERRIDE is set -- '
                         'using an overridden fingerprint (test mode only)\n')

    try:
        with open(manifest_path, 'r') as f:
            manifest = json.load(f)
        with open(engine_path, 'rb') as f:
            engine_blob = f.read()
    except OSError as e:
        _fatal(f'The activation package is missing or unreadable ({e}). The image was not '
               'built with a valid activation step.', mid, present)

    build_id = manifest.get('build_id', '')
    licensed_hint = manifest.get('machine_id')
    wraps = manifest.get('wraps', [])

    ChaCha20Poly1305, HKDF, hashes = _load_cryptography()

    if not unique:
        _fatal('No usable hardware identifiers are readable on this machine, so the engine '
               'cannot be unlocked.', mid, present, licensed_hint)

    content_key = None
    for w in wraps:
        names = w.get('names', [])
        # rebuild the subset from LOCAL values; skip a wrap we cannot fully cover
        if not names or any(n not in unique for n in names):
            continue
        subset = {n: unique[n] for n in names}
        import base64
        try:
            wrap_blob = base64.b64decode(w['wrap'])
        except Exception:
            continue
        key = _subset_key(HKDF, hashes, build_id, subset)
        K = _aead_open(ChaCha20Poly1305, key, wrap_blob)
        if K is not None and len(K) == wire.KEY_LEN:
            content_key = K
            break

    if content_key is None:
        _fatal('This machine does not match the machine the engine was licensed to '
               '(no key matched).', mid, present, licensed_hint)

    plaintext = _aead_open(ChaCha20Poly1305, content_key, engine_blob)
    if plaintext is None:
        _fatal('The engine payload failed to decrypt (corrupt activation package).',
               mid, present, licensed_hint)

    # Point the engine at its signed licence (defence in depth): the `licensed` engine build
    # re-verifies it in-process and aborts if it does not match this machine, so even this
    # decrypted image refuses to run elsewhere. A dev/unlicensed engine ignores it. Only set the
    # env if we have not been told otherwise, and only if the file exists.
    lic_path = os.path.join(activation_dir, 'license.json')
    if os.path.exists(lic_path) and not os.environ.get('METRO_LICENSE') and not os.environ.get('METRO_LICENSE_FILE'):
        os.environ['METRO_LICENSE_FILE'] = lic_path

    fd = _anon_fd(plaintext)
    # keep the fd alive for the life of the process; ctypes.CDLL dlopens the path below
    resolve_engine._held_fds.append(fd)
    return f'/proc/self/fd/{fd}'


resolve_engine._held_fds = []


def _anon_fd(data):
    """Write `data` to an anonymous in-memory fd; return the fd. memfd first, tmpfs fallback.

    Nothing is written to the image filesystem in the memfd path. The tmpfs fallback file is
    created 0600 and unlinked immediately, so it has no name on disk either.
    """
    # Preferred: memfd_create (Linux >= 3.17). Available via os on 3.8+, else via ctypes.
    fd = None
    try:
        if hasattr(os, 'memfd_create'):
            fd = os.memfd_create('metro_engine', getattr(os, 'MFD_CLOEXEC', 0))
    except Exception:
        fd = None
    if fd is None:
        fd = _memfd_via_ctypes()
    if fd is not None:
        _write_all(fd, data)
        os.lseek(fd, 0, os.SEEK_SET)
        return fd
    # Fallback: a private tmpfs file, unlinked at once so no path persists.
    return _tmpfs_fd(data)


def _memfd_via_ctypes():
    try:
        libc = ctypes.CDLL(ctypes.util.find_library('c') or 'libc.so.6', use_errno=True)
        libc.memfd_create.restype = ctypes.c_int
        libc.memfd_create.argtypes = [ctypes.c_char_p, ctypes.c_uint]
        fd = libc.memfd_create(b'metro_engine', 1)  # MFD_CLOEXEC = 1
        if fd < 0:
            return None
        return fd
    except Exception:
        return None


def _tmpfs_fd(data):
    for base in ('/dev/shm', '/run', '/tmp'):
        try:
            path = os.path.join(base, f'.metro_{os.getpid()}_{id(data) & 0xffffff}')
            fd = os.open(path, os.O_RDWR | os.O_CREAT | os.O_EXCL | os.O_CLOEXEC, 0o600)
            os.unlink(path)  # remove the name immediately; the open fd keeps the inode
            _write_all(fd, data)
            os.lseek(fd, 0, os.SEEK_SET)
            return fd
        except OSError:
            continue
    raise ActivationError('could not create an in-memory file for the engine (no writable '
                          'tmpfs). Contact ' + CONTACT)


def _write_all(fd, data):
    view = memoryview(data)
    while view:
        written = os.write(fd, view)
        view = view[written:]
