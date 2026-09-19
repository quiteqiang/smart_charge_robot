import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src" / "smart_charge_mission"))

try:
    import rclpy  # noqa: F401
except ImportError:
    collect_ignore = ["test_integration.py"]
