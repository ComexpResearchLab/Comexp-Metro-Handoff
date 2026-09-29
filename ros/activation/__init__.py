"""Machine-binding activation for the metro obstacle detector.

Shared by build time (activate.py, run as a script) and run time (loader.py, imported by the
node as the `activation` package). fingerprint.py and wire.py are the single definitions used by
both, so the build and the run can never disagree.
"""
