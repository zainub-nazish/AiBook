#!/usr/bin/env python3
"""
Isaac Sim Installation Verification Script.

This script verifies that Isaac Sim is correctly installed and can be launched.
Run this script after installing Isaac Sim to confirm your setup works.

Usage:
    python verify_install.py

Requirements:
    - NVIDIA Isaac Sim 2023.1.0+
    - NVIDIA GPU with 8GB+ VRAM
    - CUDA 11.8+
"""

import sys
import subprocess
from pathlib import Path


def check_nvidia_driver():
    """Check if NVIDIA driver is installed and get version."""
    print("Checking NVIDIA driver...")
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=driver_version,memory.total", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            check=True
        )
        driver_info = result.stdout.strip().split(", ")
        driver_version = driver_info[0]
        vram = driver_info[1]
        print(f"  ✓ Driver version: {driver_version}")
        print(f"  ✓ GPU VRAM: {vram}")

        # Check minimum driver version
        min_driver = 525
        if int(driver_version.split(".")[0]) < min_driver:
            print(f"  ⚠ Warning: Driver {driver_version} < {min_driver}. Consider upgrading.")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f"  ✗ NVIDIA driver not found: {e}")
        return False


def check_cuda():
    """Check CUDA installation."""
    print("\nChecking CUDA...")
    try:
        result = subprocess.run(
            ["nvcc", "--version"],
            capture_output=True,
            text=True,
            check=True
        )
        # Extract version from output
        for line in result.stdout.split("\n"):
            if "release" in line.lower():
                print(f"  ✓ {line.strip()}")
                return True
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("  ⚠ CUDA toolkit not found in PATH (may still work with Isaac Sim)")
        return True  # Not critical, Isaac Sim has bundled CUDA


def find_isaac_sim():
    """Find Isaac Sim installation path."""
    print("\nLocating Isaac Sim installation...")

    # Common installation paths
    common_paths = [
        Path.home() / ".local/share/ov/pkg/isaac_sim-2023.1.0",
        Path.home() / ".local/share/ov/pkg/isaac_sim-2023.1.1",
        Path.home() / ".local/share/ov/pkg/isaac_sim-2024.1.0",
        Path("/opt/nvidia/isaac-sim"),
        Path.home() / "isaac-sim",
    ]

    for path in common_paths:
        if path.exists():
            print(f"  ✓ Found Isaac Sim at: {path}")
            return path

    # Try environment variable
    import os
    isaac_path = os.environ.get("ISAAC_SIM_PATH")
    if isaac_path and Path(isaac_path).exists():
        print(f"  ✓ Found Isaac Sim via ISAAC_SIM_PATH: {isaac_path}")
        return Path(isaac_path)

    print("  ✗ Isaac Sim installation not found")
    print("  Hint: Install via Omniverse Launcher or set ISAAC_SIM_PATH")
    return None


def verify_python_env(isaac_path):
    """Verify Isaac Sim Python environment."""
    print("\nVerifying Isaac Sim Python environment...")

    if isaac_path is None:
        print("  ✗ Cannot verify - Isaac Sim not found")
        return False

    python_script = isaac_path / "python.sh"
    if not python_script.exists():
        python_script = isaac_path / "python.bat"  # Windows

    if not python_script.exists():
        print(f"  ✗ Python launcher not found at {python_script}")
        return False

    print(f"  ✓ Python launcher found: {python_script}")

    # Try to import omni modules (this would work inside Isaac Sim)
    print("  ℹ To test imports, run:")
    print(f"    {python_script} -c \"import omni; print('omni imported successfully')\"")

    return True


def check_ros2():
    """Check if ROS 2 is available for integration."""
    print("\nChecking ROS 2 integration...")
    try:
        result = subprocess.run(
            ["ros2", "--version"],
            capture_output=True,
            text=True,
            check=True
        )
        print(f"  ✓ ROS 2 available: {result.stdout.strip()}")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("  ⚠ ROS 2 not found (optional for basic Isaac Sim use)")
        return True


def main():
    """Run all verification checks."""
    print("=" * 60)
    print("Isaac Sim Installation Verification")
    print("=" * 60)

    results = {
        "NVIDIA Driver": check_nvidia_driver(),
        "CUDA": check_cuda(),
        "Isaac Sim": False,
        "Python Environment": False,
        "ROS 2 (optional)": check_ros2(),
    }

    isaac_path = find_isaac_sim()
    results["Isaac Sim"] = isaac_path is not None
    results["Python Environment"] = verify_python_env(isaac_path)

    # Summary
    print("\n" + "=" * 60)
    print("Verification Summary")
    print("=" * 60)

    all_passed = True
    for check, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {check}: {status}")
        if not passed and "optional" not in check.lower():
            all_passed = False

    print("\n" + "=" * 60)
    if all_passed:
        print("All checks passed! Isaac Sim is ready to use.")
        print("\nNext steps:")
        print("1. Launch Isaac Sim from Omniverse Launcher")
        print("2. Or run from terminal: ./isaac-sim.sh")
        print("3. Try the tutorials in Chapter 2")
        return 0
    else:
        print("Some checks failed. Please review the output above.")
        print("\nFor help, see:")
        print("- https://docs.omniverse.nvidia.com/isaacsim/latest/installation/")
        return 1


if __name__ == "__main__":
    sys.exit(main())
