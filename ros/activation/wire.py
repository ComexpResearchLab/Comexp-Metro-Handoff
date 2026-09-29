"""The activation wire format: the ONE definition shared by the (TypeScript) server and the
(Python) loader. If you change anything here, change lib/metro/wire.ts on the server to match,
or a stand will refuse its own engine.

Scheme (see ros/activation/README and the design):
  * The server picks a random 32-byte content key K and encrypts the engine .so exactly once
    with an AEAD under K (engine.enc = nonce || ciphertext+tag).
  * For every subset S of the present unique parts with |S| >= 2, and for the full set (and, if
    only one unique part exists, that single part -- a WEAK binding), the server derives
        subset_key = HKDF-SHA256(salt=build_id, ikm=SUBSET_IKM(S), info=HKDF_INFO, L=32)
    and ships wrap = AEAD(subset_key, nonce_j, K).
  * At run time the loader recomputes the local unique parts and, for each wrap, rebuilds the
    subset key from the LOCAL values of that wrap's part names (only if it has all of them),
    tries to open the wrap, and the first that authenticates yields K. No wrap opens -> refuse.

Why subsets: a disk swap (nvme changes) still leaves {uuid, board}, so a wrap over that subset
still opens. A different machine shares none of the real values, so no subset key matches.

AEAD: ChaCha20-Poly1305, 12-byte random nonce. Available in Python `cryptography` and in Node's
built-in crypto, so both ends agree without extra deps on the stand.
"""

AEAD = 'chacha20poly1305'
NONCE_LEN = 12
KEY_LEN = 32
HKDF_INFO = b'metro-activation-subset-key-v1'


def subset_ikm(subset):
    """Input keying material for one subset: 'name=value' lines, names sorted, LF-joined, utf-8.

    subset: dict {name: value}. This must be byte-identical on server and client.
    """
    return '\n'.join(f'{name}={subset[name]}' for name in sorted(subset)).encode('utf-8')
