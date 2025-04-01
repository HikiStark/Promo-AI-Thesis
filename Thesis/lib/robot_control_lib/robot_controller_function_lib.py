from lib.setup_import_standart import *


def get_prim_world_transform(prim):
    """
    Returns the 4x4 world transform matrix of the given prim as a NumPy array.
    """
    xformable = UsdGeom.Xformable(prim)
    # Compute the transform at the default time code (frame)
    transform_gf = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # Convert pxr.Gf.Matrix4d to a NumPy array
    transform_np = np.array(transform_gf)
    print("Transform Matrix for prim", prim.GetPath(), ":", transform_np)
    log_robot_message("Transform Matrix for prim " + str(prim.GetPath()) + ":\n" + str(transform_np))
    return transform_np


def matrix_to_quat(transform_np):
    """
    Extracts the rotation (3x3) from a 4x4 matrix and converts it to a quaternion [x, y, z, w].
    """
    rot_mat = transform_np[:3, :3]  # top-left 3x3 portion is rotation
    r = R.from_matrix(rot_mat)
    quat_xyzw = r.as_quat()  # [x, y, z, w]
    return quat_xyzw


def get_end_effector_pose(ee_prim_path):
    """
    Retrieves the end-effector's world pose as (position, orientation_quaternion).
    orientation is [x, y, z, w].
    """
    ee_prim = get_prim_at_path(ee_prim_path)
    if not ee_prim.IsValid():
        print(f"[Error] End effector prim not found at: {ee_prim_path}")
        log_robot_message("[Error] End effector prim not found at: " + str(ee_prim_path))
        return None, None
    else:
        print(f"Found end effector prim at: {ee_prim.GetPath()}")

    transform_np = get_prim_world_transform(ee_prim)
    # Position is the last column of the 4x4
    position = transform_np[3, :3]
    # Convert rotation to a quaternion
    orientation = matrix_to_quat(transform_np)
    print("Measured EE Quaternion:", orientation)
    log_message_save("Measured EE Quaternion: " + str(orientation))
    return position, orientation


def is_close_enough(current_pos, target_pos, current_ori, target_ori, pos_threshold=0.10, ori_threshold=0.10):
    # # Euclidean distance for position
    # pos_diff = np.linalg.norm(current_pos - target_pos)
    # # Quaternion difference (dot product approach)
    # dot_val = abs(np.dot(current_ori, target_ori))
    # ori_diff = 1.0 - dot_val
    # return (pos_diff < pos_threshold) and (ori_diff < ori_threshold)

    # Euclidean distance for position
    pos_diff = np.linalg.norm(current_pos - target_pos)

    # Compute the dot product between current and target quaternion
    dot_val = np.clip(np.dot(current_ori, target_ori), -1.0, 1.0)

    # Compute the angular difference in radians (angle = 2*arccos(dot))
    angle_diff = 2 * np.arccos(abs(dot_val))

    # Debug: print differences
    print(f"Position difference: {pos_diff:.4f}, Orientation difference (rad): {angle_diff:.4f}")
    log_robot_message(f"Position difference: {pos_diff:.4f}, Orientation difference (rad): {angle_diff:.4f}")

    return (pos_diff < pos_threshold) and (angle_diff < ori_threshold)


def track_task_progress(articulation_controller, controller, target_pos, target_ori, task_state):
    if task_state.get("done", False):
        return

    # Retrieve end-effector pose from the custom helper
    current_pos, current_ori = get_end_effector_pose("/World/UR10/ee_link")
    if current_pos is None:
        print("Could not get EE pose, skipping this frame.")
        log_robot_message("Could not get EE pose, skipping this frame.")
        return

    # Check thresholds
    if is_close_enough(current_pos, target_pos, current_ori, target_ori):
        task_state["done"] = True
        print("Task completed!")
    else:
        # # Not done; compute next action
        # actions = controller.forward(
        #     target_end_effector_position=target_pos,
        #     target_end_effector_orientation=target_ori,
        # )
        # articulation_controller.apply_action(actions)
        robot_move_to_target(articulation_controller, controller, target_pos, target_ori)

    log_message_save("Current EE Position: " + str(current_pos))


def robot_move_to_target(articulation_controller, controller, target_position, target_orientation):
    actions = controller.forward(
        target_end_effector_position=target_position,
        target_end_effector_orientation=target_orientation,
    )
    # Execute actions
    articulation_controller.apply_action(actions)
    log_message_save("Moving to target position: " + str(target_position) + " Target Orientation: " + str(target_orientation))
    log_message_save("Computed Actions: " + str(actions))
    # Optionally, you can add a small delay to allow for smoother motion
    # time.sleep(0.1)  # Adjust as needed
    world.step()
