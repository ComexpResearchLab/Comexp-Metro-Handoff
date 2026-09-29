#!/usr/bin/env python3
"""Build-time activation step. A `RUN` line in ros/Dockerfile executes this INSIDE
`docker build`, on the customer's stand, which has internet during the build.

It:
  1. computes this machine's fingerprint (fingerprint.py -- the same code the run-time loader
     uses, so build and run agree),
  2. POSTs the fingerprint plus the delivery token to the activation server,
  3. receives the engine, encrypted and bound to THIS machine, and writes the activation package
     (manifest.json + engine.enc) into the image at METRO_ACTIVATION_DIR.

The plaintext engine is never sent and never written: the server ships ciphertext, and only the
run-time loader (in RAM) ever holds the decrypted .so.

If the server cannot be reached, the build FAILS LOUDLY and prints the fingerprint plus manual
fallback instructions, so the customer is never left with a half-built image.

Only the Python standard library is used here (urllib), so the build stage needs no pip step for
activation itself. (The run-time loader needs `cryptography`; the Dockerfile installs it.)
"""
import base64
import json
import os
import sys
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fingerprint  # noqa: E402  (same-dir import; this file runs standalone in the build stage)

DEFAULT_DIR = '/opt/metro/activation'
CONTACT = os.environ.get('METRO_CONTACT', 'obstacle-detector@comexp.net')


def _die(msg, fp=None, code=2):
    line = '=' * 72
    out = ['', line, 'METRO ACTIVATION FAILED DURING BUILD', line, msg, '']
    if fp is not None:
        out.append('Send us EXACTLY this fingerprint and we will mail back an activation file to')
        out.append(f'drop into {DEFAULT_DIR} (manifest.json + engine.enc), then rebuild:')
        out.append('')
        out.append(json.dumps({'fingerprint': fp['unique'], 'context': fp['context'],
                               'machine_id': fingerprint.machine_id(fp['unique'])}, indent=2))
        out.append('')
        out.append(f'Contact: {CONTACT}')
    out += [line, '']
    sys.stderr.write('\n'.join(out) + '\n')
    sys.exit(code)


def main():
    server = os.environ.get('METRO_ACTIVATION_URL')
    token = os.environ.get('METRO_DELIVERY_TOKEN')
    out_dir = os.environ.get('METRO_ACTIVATION_DIR') or DEFAULT_DIR
    timeout = float(os.environ.get('METRO_ACTIVATION_TIMEOUT', '30'))

    fp = fingerprint.collect()
    mid = fingerprint.machine_id(fp['unique'])

    # Context is logged, never used for keys. Print it so the build log shows what stand this is.
    sys.stderr.write(f'metro activation: machine_id={mid}\n')
    sys.stderr.write(f'metro activation: unique parts present: {sorted(fp["unique"]) or "(none)"}\n')
    sys.stderr.write(f'metro activation: context: {fp["context"]}\n')

    if not server:
        _die('METRO_ACTIVATION_URL is not set (the Dockerfile must pass the activation server '
             'URL). No network call was attempted.', fp)
    if not token:
        _die('METRO_DELIVERY_TOKEN is not set. The delivery repo bakes a token; pass it as a '
             'build ARG if you rotated it.', fp)
    if not fp['unique']:
        _die('No usable hardware identifiers are readable on this machine, so the engine cannot '
             'be bound to it. Ensure the build runs as root with /sys mounted (the default).', fp)

    body = json.dumps({
        'delivery_token': token,
        'fingerprint': fp,
        'machine_id': mid,
        'client_version': 1,
    }).encode('utf-8')

    req = urllib.request.Request(server, data=body, method='POST',
                                 headers={'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            payload = json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as e:
        detail = e.read().decode('utf-8', 'replace')[:500]
        _die(f'The activation server rejected the request (HTTP {e.code}): {detail}', fp)
    except (urllib.error.URLError, OSError, ValueError) as e:
        _die(f'Could not reach the activation server at {server} ({e}).', fp)

    if not payload.get('ok'):
        _die(f'The activation server returned an error: {payload.get("error", "unknown")}', fp)

    manifest = payload.get('manifest')
    engine_b64 = payload.get('engine_b64')
    if not manifest or not engine_b64:
        _die('The activation server response was missing the manifest or the engine payload.', fp)

    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=2)
    with open(os.path.join(out_dir, 'engine.enc'), 'wb') as f:
        f.write(base64.b64decode(engine_b64))

    # In-engine licence (defence in depth): the `licensed` engine verifies it before running.
    # The server includes it only when its signing key is configured; a null licence is fine for
    # a loader-binding-only build, but a licensed engine build will refuse without one, so warn.
    license = payload.get('license')
    if license:
        with open(os.path.join(out_dir, 'license.json'), 'w') as f:
            json.dump(license, f)
        sys.stderr.write('metro activation: wrote %s/license.json (in-engine machine binding)\n' % out_dir)
    else:
        sys.stderr.write('metro activation: NOTE no signed licence returned; a licensed engine '
                         'build would refuse. (loader-level binding still applies.)\n')

    weak = manifest.get('weak')
    n_wraps = len(manifest.get('wraps', []))
    sys.stderr.write(f'metro activation: OK -- engine bound to machine_id={mid}, '
                     f'{n_wraps} wrap(s), engine_build_id={manifest.get("engine_build_id")}'
                     f'{" [WEAK: only one unique part]" if weak else ""}\n')
    sys.stderr.write(f'metro activation: wrote {out_dir}/manifest.json and {out_dir}/engine.enc\n')


if __name__ == '__main__':
    main()
