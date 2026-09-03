"""Pytest configuration and environment fixtures for Arm-robot_VLA."""

import sys
from pathlib import Path

# Ensure src/ and repo root are on sys.path for both direct and legacy imports
_ROOT = Path(__file__).resolve().parent.parent
_SRC = _ROOT / "src"

if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
