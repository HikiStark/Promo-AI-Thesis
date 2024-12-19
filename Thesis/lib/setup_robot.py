from lib.setup_import_standart import *


def setup_robot():
    # # Load asset root path
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        sys.exit()

    # 1.1 Load the UR10 model
    asset_path = assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
    # asset_path = assets_root_path + "omniverse://localhost/NVIDIA/Assets/Isaac/4.2/Isaac/Robots/UR10/ur10_short_suction.usd"
    # asset_path = assets_root_path + "/Isaac/4.2/Isaac/Robots/UR10/ur10_short_suction.usd"
    print(f"Robot asset path {asset_path}.\n")
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")

    # 1.2 Load the gripper
    gripper_usd = assets_root_path + "/Isaac/Robots/UR10/Props/short_gripper.usd"
    print(f"Gripper asset path {gripper_usd}.\n\n")
    add_reference_to_stage(usd_path=gripper_usd, prim_path="/World/UR10/ee_link")
    gripper = SurfaceGripper(
        # end_effector_prim_path="/World/UR10/ee_link", translate=0.1611, direction="x"
        end_effector_prim_path="/World/UR10/ee_link",
        translate=0,
        direction="x",
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


def attach_robot_to_table(robot_prim_path, table_prim_path):
    """
    Attach the robot to the table by setting the robot's base position to the table's position
    and applying physics constraints if necessary.
    """
    # Get the current stage
    # Get the current stage
    stage = get_current_stage()

    # Retrieve the robot and table prims
    robot_prim = get_prim_at_path(robot_prim_path)
    table_prim = get_prim_at_path(table_prim_path)

    if not robot_prim.IsValid():
        raise ValueError(f"Error: Robot prim at {robot_prim_path} is not valid.")
    if not table_prim.IsValid():
        raise ValueError(f"Error: Table prim at {table_prim_path} is not valid.")

    # Ensure physics attributes are disabled on the robot base to avoid conflicts
    if robot_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(robot_prim).GetRigidBodyEnabledAttr().Set(False)

    # Get the table's world transform
    table_xform = UsdGeom.Xformable(table_prim)
    table_transform = table_xform.ComputeLocalToWorldTransform(0)

    # Set the robot's position to the table's top surface
    robot_xform = UsdGeom.Xformable(robot_prim)
    table_top_position = table_transform.ExtractTranslation() + Gf.Vec3d(
        0, 0, 0.825
    )  # Adjust height

    # Reset transform operations to avoid conflicts
    robot_xform.ClearXformOpOrder()

    robot_xform.AddTranslateOp().Set(table_top_position)

    print(f"Robot attached to table at position {table_top_position}.")

    # Add a fixed joint to attach the robot base to the table
    joint_prim_path = f"{robot_prim_path}/base_joint"
    if not stage.GetPrimAtPath(joint_prim_path).IsValid():
        joint = UsdPhysics.FixedJoint.Define(stage, joint_prim_path)
        joint.CreateBody0Rel().SetTargets([table_prim.GetPath()])
        joint.CreateBody1Rel().SetTargets([robot_prim.GetPath()])
        print("Fixed joint created between the robot and the table.")

    print("Physics constraints applied to the robot.")


def attach_robot_to_table_with_relative_position(robot_prim_path, table_prim_path):
    """
    Attach the robot to the table by aligning its base position relative to the table's top surface.
    Automatically calculates the table's dimensions and sets the robot's position accordingly.

    Args:
        robot_prim_path (str): The prim path of the robot in the USD stage.
        table_prim_path (str): The prim path of the table in the USD stage.

    Returns:
        None
    """
    # Get the current stage
    stage = get_current_stage()

    # Retrieve the robot and table prims
    robot_prim = get_prim_at_path(robot_prim_path)
    table_prim = get_prim_at_path(table_prim_path)

    if not robot_prim.IsValid():
        raise ValueError(f"Error: Robot prim at {robot_prim_path} is not valid.")
    if not table_prim.IsValid():
        raise ValueError(f"Error: Table prim at {table_prim_path} is not valid.")

    # Get the table's bounding box to calculate the top surface position
    table_geom = UsdGeom.Boundable(table_prim)
    bbox = table_geom.ComputeWorldBound(0, "default")
    table_min = bbox.GetRange().GetMin()
    table_max = bbox.GetRange().GetMax()

    # Calculate the top center of the table
    table_top_position = Gf.Vec3d(
        (table_min[0] + table_max[0]) / 2,  # Center along X
        (table_min[1] + table_max[1]) / 2,  # Center along Y
        table_max[2],  # Top surface Z
    )

    # Adjust robot's base to align with the table's top
    robot_xform = UsdGeom.Xformable(robot_prim)

    # Ensure a reset for xform ops to avoid conflicts
    robot_xform.ClearXformOpOrder()
    print("Cleared existing transform operations before setting new translation.")

    robot_xform.AddTranslateOp().Set(table_top_position)

    print(f"Robot attached to table at position {table_top_position}.")

    # Optionally apply physics constraints to fix the robot to the table
    if not robot_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(robot_prim)

    # Add a joint to attach the robot base to the table
    joint_prim_path = f"{robot_prim_path}/base_joint"
    if not stage.GetPrimAtPath(joint_prim_path).IsValid():
        joint = UsdPhysics.FixedJoint.Define(stage, joint_prim_path)
        joint.CreateBody0Rel().SetTargets([table_prim.GetPath()])
        joint.CreateBody1Rel().SetTargets([robot_prim.GetPath()])
        print("Fixed joint created between the robot and the table.")

    print("Physics constraints applied to the robot.")


def set_robot_attach_table():
    robot_prim_path = "/World/UR10"
    table_prim_path = "/World/Table"
    # attach_robot_to_table(robot_prim_path, table_prim_path)
    attach_robot_to_table_with_relative_position(robot_prim_path, table_prim_path)
