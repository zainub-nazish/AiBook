#!/usr/bin/env python3
"""
Domain Randomization Script for Isaac Sim.

This script demonstrates how to use the Replicator API to apply
domain randomization to a scene for generating diverse training data.

Randomization includes:
    - Lighting intensity and color
    - Object positions and rotations
    - Material textures and colors
    - Camera parameters

Usage:
    Run from within Isaac Sim Python environment:
    ./python.sh domain_randomizer.py

Requirements:
    - NVIDIA Isaac Sim 2023.1.0+
    - Replicator extension enabled
"""

from omni.isaac.kit import SimulationApp

CONFIG = {
    "headless": True,
    "width": 1280,
    "height": 720,
}

simulation_app = SimulationApp(CONFIG)

import omni
import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from pxr import UsdLux
from pathlib import Path
import numpy as np


OUTPUT_DIR = Path("./output_randomized")


def setup_output_directories():
    """Create output directories."""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"✓ Output directory: {OUTPUT_DIR.absolute()}")


def create_base_scene():
    """Create the base scene to be randomized."""
    stage = omni.usd.get_context().get_stage()
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Create target objects that will be randomized
    for i in range(5):
        DynamicCuboid(
            prim_path=f"/World/Objects/Cube_{i}",
            name=f"cube_{i}",
            position=[1.0 + i * 0.3, 0, 0.1],
            scale=[0.1, 0.1, 0.1],
            color=[0.5, 0.5, 0.5],
        )

    # Add light
    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.GetIntensityAttr().Set(1000)

    world.reset()
    print("✓ Base scene created")
    return world


def setup_lighting_randomization():
    """Configure lighting randomization."""
    # Get the dome light
    lights = rep.get.prim_at_path("/World/DomeLight")

    with lights:
        rep.modify.attribute(
            "intensity",
            rep.distribution.uniform(500, 2000)  # Random intensity
        )

    print("✓ Lighting randomization configured")
    print("  - Intensity: uniform(500, 2000)")


def setup_object_randomization():
    """Configure object pose randomization."""
    # Get all cubes
    cubes = rep.get.prims(path_pattern="/World/Objects/Cube_*")

    with cubes:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (0.5, -0.5, 0.05),
                (2.0, 0.5, 0.3)
            ),
            rotation=rep.distribution.uniform(
                (0, 0, 0),
                (0, 0, 360)
            ),
        )

    print("✓ Object pose randomization configured")
    print("  - Position: uniform in workspace")
    print("  - Rotation: random yaw")


def setup_material_randomization():
    """Configure material/texture randomization."""
    cubes = rep.get.prims(path_pattern="/World/Objects/Cube_*")

    with cubes:
        rep.randomizer.color(
            colors=rep.distribution.uniform(
                (0.0, 0.0, 0.0),
                (1.0, 1.0, 1.0)
            )
        )

    print("✓ Material randomization configured")
    print("  - Color: random RGB")


def setup_camera_randomization():
    """Create camera with randomized parameters."""
    camera = rep.create.camera(
        position=rep.distribution.uniform(
            (2.0, -0.5, 1.0),
            (3.0, 0.5, 2.0)
        ),
        look_at=(1.0, 0, 0.15),
        focal_length=rep.distribution.uniform(20, 35),
    )

    print("✓ Camera randomization configured")
    print("  - Position: varied viewpoint")
    print("  - Focal length: 20-35mm")

    return camera


def setup_writer(camera):
    """Configure data writer."""
    render_product = rep.create.render_product(camera, (1280, 720))

    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir=str(OUTPUT_DIR),
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
    )
    writer.attach([render_product])

    print("✓ Writer configured")
    print("  - RGB images")
    print("  - 2D bounding boxes")
    print("  - Semantic segmentation")

    return writer


def generate_randomized_data(num_frames=100):
    """Generate frames with domain randomization."""
    print(f"\nGenerating {num_frames} randomized frames...")

    for i in range(num_frames):
        # Each step applies new randomization
        rep.orchestrator.step()

        if (i + 1) % 10 == 0:
            print(f"  Progress: {i+1}/{num_frames}")

    print(f"✓ Generated {num_frames} randomized frames")


def main():
    """Main function for domain randomization."""
    print("\n" + "=" * 50)
    print("Domain Randomization for Synthetic Data")
    print("=" * 50 + "\n")

    setup_output_directories()
    world = create_base_scene()

    # Configure randomizers
    print("\nConfiguring randomization...")
    print("-" * 30)
    setup_lighting_randomization()
    setup_object_randomization()
    setup_material_randomization()
    camera = setup_camera_randomization()
    writer = setup_writer(camera)

    # Generate data
    NUM_FRAMES = 100
    generate_randomized_data(NUM_FRAMES)

    # Summary
    print("\n" + "=" * 50)
    print("Domain Randomization Complete!")
    print("=" * 50)
    print(f"\nOutput saved to: {OUTPUT_DIR.absolute()}")
    print(f"\nGenerated {NUM_FRAMES} frames with variations in:")
    print("  - Lighting intensity")
    print("  - Object positions and rotations")
    print("  - Object colors")
    print("  - Camera viewpoint and focal length")
    print("\nThis diverse dataset helps train models that")
    print("generalize better from simulation to real world.")

    simulation_app.close()


if __name__ == "__main__":
    main()
