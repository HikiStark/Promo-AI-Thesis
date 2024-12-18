from lib.setup_import_standart import *

from omni.isaac.core.utils.stage import add_reference_to_stage
from lib.setup_camera import camera_add, camera_add_overhead
from pxr import UsdGeom

world = World(stage_units_in_meters=1.0)


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
    add_reference_to_stage(  # Add a pre-existing table USD file
        usd_path="omniverse://localhost/NVIDIA/Assets/Props/Furniture/Table.usd",
        prim_path="/World/Table",
    )

    # Position the table
    table_prim = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Table"))
    table_prim.AddTranslateOp().Set(value=(0.0, 0.0, 0.5))  # Position the table


def add_cubes():  # Add some cubes on top of the table. Each cube is small, say 0.05m on a side.
    cube_positions = [(0.0, 0.0, 0.525), (0.1, 0.1, 0.525), (-0.1, 0.1, 0.525)]
    for i, pos in enumerate(cube_positions):
        cube = DynamicCuboid(
            prim_path=f"/World/Cube_{i}",
            name=f"cube_{i}",
            position=pos,
            size=(0.05, 0.05, 0.05),
            color=(1.0, 0.0, 0.0),  # Red cubes
        )
        world.scene.add(cube)


class set_the_scene:
    world.scene.add_default_ground_plane()  # add ground plane
    add_table_usd()
    # add_cubes()
    # camera
