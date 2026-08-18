"""Pytest bootstrap for the Robotic Harness worker test suite.

Loads cv2 (and the other native libraries) in a fixed order BEFORE any test
runs. On Windows, OpenCV's native runtime is known to conflict with other
native libraries (mujoco, pyarrow, matplotlib) when they get loaded in
different orders — a conflict that can hard-crash the interpreter inside
``cv2.cvtColor`` (see ``vision._cv2_failure`` docs). Importing cv2 first makes
the load order deterministic.
"""

import importlib  # noqa: F401

for _module in ("cv2", "numpy", "mujoco", "matplotlib", "pyarrow", "PIL"):
    try:
        importlib.import_module(_module)
    except Exception:  # noqa: BLE001 - optional deps
        pass
