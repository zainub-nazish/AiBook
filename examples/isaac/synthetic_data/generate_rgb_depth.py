#!/usr/bin/env python3
"""
RGB and Depth Image Generation Script for Isaac Sim.

This script demonstrates how to generate RGB and depth images from
a scene in Isaac Sim using the Replicator API.

Usage:
    Run from within Isaac Sim Python environment:
    ./python.sh generate_rgb_depth.py

Output:
    - RGB images: output/rgb/rgb_XXXX.png
    - Depth images: output/depth/depth_XXXX.exr
    - Distance images: output/distance/distance_XXXX.exr

Requirements:
    - NVIDIA Isaac Sim 2023.1.0+
    - Scene must be set up with camera
"""

from omni.isaac.kit import SimulationApp

CONFIG = {
    "headless": True,  # Run without GUI for faster generation
    "width": 1280,
    "height": 720,
}

simulation_app = SimulationApp(CONFIG)

import omni
import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from pxr import Gf, UsdGeom
import numpy as np
from pathlib import Path


# Output directory
OUTPUT_DIR = Path("./output")


def setup_output_directories():
    """Create output directories for generated data."""
    dirs = ["rgb", "depth", "distance", "normals"]
    for d in dirs:
        (OUTPUT_DIR / d).mkdir(parents=True, exist_ok=True)
    print(f"✓ Output directories created at: {OUTPUT_DIR.absolute()}")


def create_simple_scene():
    """Create a simple scene with objects for data generation."""
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0)

    # Add ground
    world.scene.add_default_ground_plane()

    # Add some cubes at different positions
    from omni.isaac.core.objects import DynamicCuboid

    colors = [
        [1.0, 0.0, 0.0],  # Red
        [0.0, 1.0, 0.0],  # Green
        [0.0, 0.0, 1.0],  # Blue
        [1.0, 1.0, 0.0],  # Yellow
    ]

    positions = [
        [1.0, 0.5, 0.1],
        [1.5, -0.3, 0.1],
        [0.8, -0.5, 0.1],
        [1.2, 0.0, 0.3],
    ]

    for i, (pos, color) in enumerate(zip(positions, colors)):
        DynamicCuboid(
            prim_path=f"/World/Cube_{i}",
            name=f"cube_{i}",
            position=pos,
            scale=[0.15, 0.15, 0.15],
            color=color,
        )

    # Add lighting
    from pxr import UsdLux
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.GetIntensityAttr().Set(1500)

    world.reset()
    print("✓ Scene created with 4 colored cubes")
    return world


def create_camera():
    """Create a camera for rendering."""
    # Create camera using Replicator
    camera = rep.create.camera(
        position=(2.5, 0, 1.5),
        look_at=(1.0, 0, 0.2),
        focal_length=24.0,
        f_stop=2.8,
    )
    print("✓ Camera created")
    return camera


def setup_render_products(camera):
    """Set up render products for different data types."""

    # Create render product (what the camera sees)
    render_product = rep.create.render_product(camera, (1280, 720))

    # Initialize the writer for basic outputs
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=str(OUTPUT_DIR),
        rgb=True,
        distance_to_camera=True,
        normals=True,
    )
    writer.attach([render_product])

    print("✓ Render products configured")
    print("  - RGB images")
    print("  - Distance to camera (depth)")
    print("  - Surface normals")

    return render_product, writer


def generate_data(num_frames=10):
    """Generate the specified number of frames."""
    print(f"\nGenerating {num_frames} frames...")

    for i in range(num_frames):
        # Step the replicator to generate one frame
        rep.orchestrator.step()
        print(f"  Frame {i+1}/{num_frames} generated")

    print(f"\n✓ Generated {num_frames} frames")


def main():
    """Main function to generate RGB and depth data."""
    print("\n" + "=" * 50)
    print("RGB and Depth Image Generation")
    print("=" * 50 + "\n")

    # Setup
    setup_output_directories()
    world = create_simple_scene()
    camera = create_camera()
    render_product, writer = setup_render_products(camera)

    # Generate data
    NUM_FRAMES = 10
    generate_data(NUM_FRAMES)

    # Summary
    print("\n" + "=" * 50)
    print("Generation Complete!")
    print("=" * 50)
    print(f"\nOutput saved to: {OUTPUT_DIR.absolute()}")
    print("\nGenerated files:")
    print(f"  - {NUM_FRAMES} RGB images (PNG)")
    print(f"  - {NUM_FRAMES} distance maps (NPY)")
    print(f"  - {NUM_FRAMES} normal maps (NPY)")
    print("\nTo view depth images, use:")
    print("  python -c \"import numpy as np; import matplotlib.pyplot as plt;")
    print("             d = np.load('output/distance_to_camera/distance_0001.npy');")
    print("             plt.imshow(d); plt.colorbar(); plt.show()\"")

    simulation_app.close()


if __name__ == "__main__":
    main()
