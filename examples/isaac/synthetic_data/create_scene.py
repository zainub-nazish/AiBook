#!/usr/bin/env python3
"""
Isaac Sim Scene Creation Script.

This script demonstrates how to programmatically create a scene in Isaac Sim
with a simple indoor environment, lighting, and a robot.

Usage:
    Run from within Isaac Sim Python environment:
    ./python.sh create_scene.py

Requirements:
    - NVIDIA Isaac Sim 2023.1.0+
    - Must be run using Isaac Sim's Python interpreter
"""

# Isaac Sim imports (only work inside Isaac Sim environment)
from omni.isaac.kit import SimulationApp

# Initialize the simulation application
CONFIG = {
    "headless": False,  # Set True for headless rendering
    "width": 1280,
    "height": 720,
}

simulation_app = SimulationApp(CONFIG)

# Now we can import other omni modules
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.prims import XFormPrim
from pxr import Gf, UsdLux


def create_ground_plane(world):
    """Add a ground plane to the scene."""
    world.scene.add_default_ground_plane()
    print("✓ Added ground plane")


def create_lighting():
    """Create scene lighting with dome light and area light."""
    stage = omni.usd.get_context().get_stage()

    # Create a dome light for ambient lighting
    dome_light_path = "/World/DomeLight"
    dome_light = UsdLux.DomeLight.Define(stage, dome_light_path)
    dome_light.GetIntensityAttr().Set(1000)
    dome_light.GetTextureFileAttr().Set("")  # Could add HDRI here
    print("✓ Added dome light")

    # Create an area light for directional lighting
    area_light_path = "/World/AreaLight"
    area_light = UsdLux.RectLight.Define(stage, area_light_path)
    area_light.GetIntensityAttr().Set(5000)
    area_light.GetWidthAttr().Set(2.0)
    area_light.GetHeightAttr().Set(2.0)

    # Position the area light above the scene
    xform = XFormPrim(area_light_path)
    xform.set_world_pose(position=[0, 0, 3], orientation=[0.7071, 0, 0.7071, 0])
    print("✓ Added area light")


def add_simple_room():
    """Create a simple room with walls."""
    stage = omni.usd.get_context().get_stage()

    # Create walls using cube prims
    from omni.isaac.core.objects import DynamicCuboid

    wall_thickness = 0.1
    wall_height = 2.5
    room_size = 5.0

    # Back wall
    back_wall = DynamicCuboid(
        prim_path="/World/Room/BackWall",
        name="back_wall",
        position=[0, room_size/2, wall_height/2],
        scale=[room_size, wall_thickness, wall_height],
        color=[0.8, 0.8, 0.8],
    )
    print("✓ Added room walls")


def add_table_with_objects():
    """Add a table with some objects on it."""
    from omni.isaac.core.objects import DynamicCuboid

    # Table
    table = DynamicCuboid(
        prim_path="/World/Objects/Table",
        name="table",
        position=[1.0, 0, 0.4],
        scale=[1.0, 0.6, 0.8],
        color=[0.6, 0.4, 0.2],
    )

    # Object on table (a cube representing an item)
    cube = DynamicCuboid(
        prim_path="/World/Objects/Cube",
        name="cube",
        position=[1.0, 0, 0.9],
        scale=[0.1, 0.1, 0.1],
        color=[1.0, 0.0, 0.0],
    )

    print("✓ Added table with objects")


def add_robot():
    """Add a robot model to the scene."""
    assets_root = get_assets_root_path()

    if assets_root is None:
        print("⚠ Could not find Nucleus assets root. Using local fallback.")
        return None

    # Try to load a simple robot (Carter robot from Isaac Sim assets)
    robot_usd = assets_root + "/Isaac/Robots/Carter/carter_v1.usd"

    try:
        add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Robot")
        robot_xform = XFormPrim("/World/Robot")
        robot_xform.set_world_pose(position=[0, 0, 0])
        print(f"✓ Added robot from: {robot_usd}")
        return robot_xform
    except Exception as e:
        print(f"⚠ Could not load robot: {e}")
        return None


def main():
    """Create the complete scene."""
    print("\n" + "=" * 50)
    print("Creating Isaac Sim Scene")
    print("=" * 50 + "\n")

    # Create the world
    world = World(stage_units_in_meters=1.0)
    print("✓ Created world")

    # Build the scene
    create_ground_plane(world)
    create_lighting()
    add_table_with_objects()
    robot = add_robot()

    # Reset the world to initialize all objects
    world.reset()

    print("\n" + "=" * 50)
    print("Scene created successfully!")
    print("=" * 50)
    print("\nScene hierarchy:")
    print("  /World")
    print("  ├── defaultGroundPlane")
    print("  ├── DomeLight")
    print("  ├── AreaLight")
    print("  ├── Objects/")
    print("  │   ├── Table")
    print("  │   └── Cube")
    print("  └── Robot (if loaded)")
    print("\nYou can now:")
    print("1. Navigate the scene in the viewport")
    print("2. Add sensors to the robot")
    print("3. Run simulation with Play button")

    # Keep the simulation running
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
