"""Entry point for ``python -m robotic_harness_worker``."""

import sys

from .cli import main

if __name__ == "__main__":
    sys.exit(main())
