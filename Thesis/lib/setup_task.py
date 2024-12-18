from lib.setup_import_standart import *

from omni.isaac.core.utils.stage import add_reference_to_stage
from lib.setup_camera import camera_add, camera_add_overhead
from pxr import UsdGeom

world = World(stage_units_in_meters=1.0)


# def apply_collision(prim):
#     """
#     Apply collision and physics attributes to a given prim.

#     Args:
#         prim (Usd.Prim): The USD prim to which collision will be applied.
#     """
#     # Apply collision properties
#     UsdPhysics.CollisionAPI.Apply(prim)

#     # Set collision type as convexHull for efficiency
#     collision_attr = PhysxSchema.PhysxCollisionAPI.Apply(prim)
#     collision_attr.CreateCollisionTypeAttr().Set(
#         "convexHull"
#     )  # Options: convexHull, mesh, none

#     # Enable rigid body physics
#     physics_api = UsdPhysics.RigidBodyAPI.Apply(prim)
#     physics_api.CreateRigidBodyEnabledAttr(True)

#     print(f"Collision applied to prim: {prim.GetPath()}")


def setup_robot():
    # # Load asset root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        sys.exit()

    # 1.1 Load the UR10 model
    asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")

    # 1.2 Load the gripper
    gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
    add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
    gripper = SurfaceGripper(
        end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x"
    )

    # 1.3 Initialize the robot with gripper
    ur10 = world.scene.add(
        SingleManipulator(
            prim_path="/World/UR10",
            name="my_ur10",
            end_effector_prim_path="/World/UR10/ee_link",
            gripper=gripper,
        )
    )

    # Set initial joint states
    ur10.set_joints_default_state(
        positions=np.array(
            [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]
        )
    )

    # Set default gripper state
    ur10.gripper.set_default_state(opened=True)

    # Define the tasks
    articulation_controller = ur10.get_articulation_controller()

    # RMPFlow controller for advanced movement
    my_controller_RMP = RMPFlowController(
        name="target_follower_controller", robot_articulation=ur10, attach_gripper=True
    )


def add_table():
    table = FixedCuboid(
        prim_path="/World/table",
        name="table",
        position=np.array([0.0, 0.0, 0.5]),  # Position above the ground plane
        scale=np.array([1.0, 2.0, 0.1]),  # Dimensions of the table
        color=np.array([0.7, 0.4, 0.2]),  # Color of the table
    )


def add_table_usd():
    """
    Add a table USD asset to the stage and verify it.
    """
    # Define paths
    usd_path = "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Environments/Outdoor/Rivermark/dsready_content/nv_content/common_assets/props_general/table01/table01.usd"
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
    print(f"Retrieving prim at: {sdf_path}")

    prim = stage.GetPrimAtPath(sdf_path)
    print(f"Prim object: {prim}")

    # Validate the prim
    if not prim.IsValid():
        print(f"Error: Prim at {prim_path} is not valid.")
        return

    print(f"Success: Table prim added at {prim_path}")

    apply_collision(prim)
    print(f"Success: Table prim added and collision applied at {sdf_path}")


def apply_collision(prim):
    """
    Apply collision and physics properties to a given prim.

    Args:
        prim (Usd.Prim): The USD prim to which collision will be applied.
    """
    # Apply the collision API
    collision_api = UsdPhysics.CollisionAPI.Apply(prim)
    print("Collision API applied.")

    # Set physics attributes
    collision_type_attr = collision_api.GetCollisionEnabledAttr()
    if not collision_type_attr:
        collision_api.CreateCollisionEnabledAttr(True)
        print("Collision enabled attribute created.")

    # Add physics mass (optional for dynamic bodies)
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
        print("Rigid body API applied to enable physics.")

    print("Collision successfully applied to prim.")


def add_cube_nvidia():
    add_reference_to_stage(  # Add a pre-existing table USD file
        usd_path="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Props/Blocks/nvidia_cube.usd",
        prim_path="/World/cube",
    )


def add_cube_dex():
    add_reference_to_stage(  # Add a pre-existing table USD file
        usd_path="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Props/Blocks/DexCube/dex_cube_instanceable.usd",
        prim_path="/World/cube",
    )


def add_cube_multicolor():
    add_reference_to_stage(  # Add a pre-existing table USD file
        usd_path="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Props/Blocks/MultiColorCube/multi_color_cube_instanceable.usd",
        prim_path="/World/cube",
    )


def add_cubes():  # Add some cubes on top of the table. Each cube is small, say 0.05m on a side.
    cube_positions = [(0.0, 0.0, 0.525), (0.1, 0.1, 0.525), (-0.1, 0.1, 0.525)]
    for i, pos in enumerate(cube_positions):
        add_cube_nvidia()


def set_the_scene():
    world.scene.add_default_ground_plane()  # add ground plane
    add_table_usd()
    add_cubes()
    # camera
