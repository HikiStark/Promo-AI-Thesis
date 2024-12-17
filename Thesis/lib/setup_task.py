from lib.setup_import_standart import *

# Create a World
world = World(stage_units_in_meters=1.0)

# Parse arguments for test mode
parser = argparse.ArgumentParser()
parser.add_argument(
    "--test", default=False, action="store_true", help="Run in test mode"
)
args, unknown = parser.parse_known_args()

# Load asset root path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

class setup_robot:
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


# ---------------------------------------------------------------------------------------------------------------
# Create a "Table" using a Static Cuboid
# Let's say our table surface is 1 meter by 1 meter, and 0.05m thick.
class add_table:
    table = VisualCuboid(
        prim_path="/World/Table",
        name="table",
        position=(0.0, 0.0, 0.5),  # This elevates the table top at z=0.5m
        size=(1.0, 1.0, 0.05),  # (X, Y, Z) size
        color=(0.6, 0.3, 0.0),  # Brownish color
    )

    world.scene.add(table)


# Add some cubes on top of the table
# Each cube is small, say 0.05m on a side.
class add_cubes:
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

camera = camera_add()

class set_the_scene:
    world.scene.add_default_ground_plane()  # add ground plane
    add_table()
    add_cubes()
    camera_add()

world.reset()
for _ in range(10):
    world.step(render=True)
    camera.get_rgb_image()