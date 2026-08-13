"""Run the worker test suite with per-file process isolation.

Why: some test files load native libraries (mujoco, cv2/OpenCV, pyarrow)
that can conflict when combined in one process on Windows (known OpenCV
"Unknown C++ exception" DLL-collision). Each test file runs in its own
fresh interpreter, which mirrors production where every worker command is a
one-shot process.

Usage:
    python run_tests.py [--file tests/test_x.py ...]
"""

from __future__ import annotations

import argparse
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main() -> int:
    parser = argparse.ArgumentParser(description="Run worker tests per file (isolated processes)")
    parser.add_argument("--file", action="append", help="specific test file(s); default: all tests/*.py")
    args = parser.parse_args()

    if args.file:
        files = [os.path.join(HERE, f) if not os.path.isabs(f) else f for f in args.file]
    else:
        files = sorted(glob.glob(os.path.join(HERE, "tests", "test_*.py")))

    failures: list[str] = []
    for index, path in enumerate(files, start=1):
        name = os.path.basename(path)
        print(f"\n===== [{index}/{len(files)}] {name} =====", flush=True)
        result = subprocess.run([sys.executable, "-m", "pytest", path, "-q"], cwd=HERE)
        if result.returncode != 0:
            failures.append(name)

    print("\n===== summary =====")
    print(f"files: {len(files)}, failed: {len(failures)}")
    for name in failures:
        print(f"  FAILED: {name}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
