from lib.setup_import_standart import *
from lib.setup_camera import add_camera_to_scene
from lib.setup_robot import attach_robot_to_table, set_robot_attach_table

# # Load asset root path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    sys.exit()

def add_table():
    """
    Add a table USD asset to the stage and verify it.
    """
    # Define paths
    usd_path = assets_root_path + "/Isaac/Environments/Outdoor/Rivermark/dsready_content/nv_content/common_assets/props_general/table01/table01.usd"
    prim_path = "/World/Table"

    print("Adding table USD asset...")

    # Add USD to the stage
    add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

    # Get the current stage
    stage = get_current_stage()
    print(f"Stage object: {stage}")

    # Debug: Check if the stage object is valid
    if not isinstance(stage, Usd.Stage):
        raise RuntimeError("Error: Failed to get a valid stage object.")

    # Get the prim at the specified path
    sdf_path = Sdf.Path(prim_path)  # Convert prim_path to Sdf.Path
    print(f"\n\nRetrieving prim at: {sdf_path}")

    prim = stage.GetPrimAtPath(sdf_path)
    print(f"Prim object: {prim}")

    # Validate the prim
    if not prim.IsValid():
        print(f"Error: Prim at {prim_path} is not valid.")
        return

    print(f"\nSuccess: Table prim added at {prim_path}")

    apply_collision(prim)
    print(f"\nSuccess: Table prim added and collision applied at {sdf_path}")


def apply_collision(prim):
    """
    Apply collision and physics properties to a given prim.

    Args:
        prim (Usd.Prim): The USD prim to which collision will be applied.
    """
    # Apply the collision API
    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    print("\nCollision API applied.")

    # Set physics attributes
    collision_type_attr = collision_api.GetCollisionEnabledAttr()
    if not collision_type_attr:
        collision_api.CreateCollisionEnabledAttr(True)
        print("Collision enabled attribute created.\n")

    # Add physics mass (optional for dynamic bodies)
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
        print("Rigid body API applied to enable physics.\n")

    print("Collision successfully applied to prim.\n\n")


def add_cube(index, position):
    """
    Add a cube with a unique name at the specified position.

    Args:
        index (int): Unique index for naming the cube.
        position (tuple): (x, y, z) position to place the cube.
    """
    prim_path = f"/World/cube_{index}"  # Unique path for each cube
    add_reference_to_stage(
        usd_path= assets_root_path + "/Isaac/Props/Blocks/nvidia_cube.usd",
        # usd_path= assets_root_path + "/Isaac/Props/Blocks/MultiColorCube/multi_color_cube_instanceable.usd", # multicolor cube
        # usd_path= assets_root_path + "/Isaac/Props/Blocks/DexCube/dex_cube_instanceable.usd", # dex cube
        prim_path=prim_path,
    )

    # Move cube to the desired position
    cube_prim = get_prim_at_path(prim_path)
    if not cube_prim.IsValid():
        print(f"\nError: Cube at {prim_path} is invalid.")
        return

    UsdGeom.Xformable(cube_prim).AddTranslateOp().Set(position)
    print(f"\n\nCube {index} added at position {position} \n\n")


def add_cubes():
    """
    Add multiple cubes at different positions.
    """
    cube_positions = [(0.0, 0.0, 0.825), (0.1, 0.1, 0.825), (-0.1, 0.1, 0.825)]
    for i, pos in enumerate(cube_positions):
        add_cube(i, pos)  # Pass unique index and position


def set_the_scene():
    world.scene.add_default_ground_plane()  # add ground plane
    add_table()
    add_cubes()
    camera = add_camera_to_scene(
        prim_path="/World/OverheadCamera",
        position=(2.0, 3.0, 5.0),
        orientation=(1, 0, 0, 0),  # Identity quaternion
        resolution=(1280, 720),
        frequency=30,
    )
    set_robot_attach_table()