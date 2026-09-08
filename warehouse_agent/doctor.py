"""Read-only observations, deliberately separate from simulation smoke tests."""

from datetime import datetime, timezone
import importlib.util
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def inspect_environment() -> dict:
    release = platform.freedesktop_os_release()
    gpu = shutil.which("nvidia-smi")
    if not gpu and Path("/usr/lib/wsl/lib/nvidia-smi").exists():
        gpu = "/usr/lib/wsl/lib/nvidia-smi"
    gpu_result = {"status": "unavailable", "detail": "nvidia-smi not found"}
    if gpu:
        try:
            probe = subprocess.run(
                [gpu, "--query-gpu=name,memory.total,driver_version", "--format=csv,noheader"],
                capture_output=True, text=True, timeout=10, check=False,
            )
            gpu_result = {
                "status": "observed" if probe.returncode == 0 else "probe_failed",
                "detail": (probe.stdout + probe.stderr).strip(),
            }
        except (OSError, subprocess.TimeoutExpired) as exc:
            gpu_result = {"status": "probe_failed", "detail": str(exc)}
    noble = release.get("ID") == "ubuntu" and release.get("VERSION_ID") == "24.04"
    jazzy = Path("/opt/ros/jazzy/setup.bash").exists()
    tools = {name: shutil.which(name) for name in ("git", "ros2", "gz", "colcon", "docker")}
    return {
        "observed_at_utc": datetime.now(timezone.utc).isoformat(),
        "os": release.get("PRETTY_NAME"),
        "kernel": platform.release(),
        "python": platform.python_version(),
        "python_executable": sys.executable,
        "virtual_environment": sys.prefix != sys.base_prefix,
        "pip_available": importlib.util.find_spec("pip") is not None,
        "tools": tools,
        "display": {key: os.environ.get(key) for key in ("DISPLAY", "WAYLAND_DISPLAY")},
        "gpu": gpu_result,
        "readiness": {
            "core_python_version_satisfied": sys.version_info >= (3, 12),
            "selected_ros_platform_satisfied": noble,
            "jazzy_setup_file_present": jazzy,
            "simulation_smoke_test": "not assessed by this command",
        },
        "next_step": (
            "Follow docs/environment.md for stock Nav2 navigation and cancellation checks."
            if noble else
            "Core work can proceed here. Use a separate Ubuntu 24.04 distribution for Jazzy."
        ),
    }
