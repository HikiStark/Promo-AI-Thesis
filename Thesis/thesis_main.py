from isaacsim import SimulationApp

# Create and configure the simulation app.
simulation_app = SimulationApp(
    {
        "headless": False,
        # "window_width": 2400,
        # "window_height": 1500,
    }
)
# Import required modules after initializing the simulation app.
import zmq
import json
from lib.setup_import_standart import *
from lib.setup_robot import setup_robot
import lib.setup_task as tasksetup
from lib.tasks.robot_look_table import robot_look_at_table

# Set up ZMQ publisher
context = zmq.Context()

print("-" * 120 + "\n")

articulation_controller, my_controller_RMP, my_controller_PP = setup_robot()
world.reset()
world.step()
scene = tasksetup.set_the_scene(simulation_app)
camera_instance = scene.camera
print("camera_instance:", camera_instance)

print("\n" + "-" * 120)


def initialize_publisher():
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 1)  # Limit backlog
    socket.bind("tcp://*:5555")
    return socket


def initialize_command_receiver():
    cmd_socket = context.socket(zmq.PULL)
    cmd_socket.bind("tcp://*:5560")  # Bind on a new port (5560)
    return cmd_socket


def get_prim_world_transform(prim):
    """
    Returns the 4x4 world transform matrix of the given prim as a NumPy array.
    """
    xformable = UsdGeom.Xformable(prim)
    # Compute the transform at the default time code (frame)
    transform_gf = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    # Convert pxr.Gf.Matrix4d to a NumPy array
    transform_np = np.array(transform_gf)
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
        return None, None

    transform_np = get_prim_world_transform(ee_prim)
    # Position is the last column of the 4x4
    position = transform_np[:3, 3]
    # Convert rotation to a quaternion
    orientation = matrix_to_quat(transform_np)
    return position, orientation


def is_close_enough(current_pos, target_pos, current_ori, target_ori, pos_threshold=0.03, ori_threshold=0.03):
    # Euclidean distance for position
    pos_diff = np.linalg.norm(current_pos - target_pos)
    # Quaternion difference (dot product approach)
    dot_val = abs(np.dot(current_ori, target_ori))
    ori_diff = 1.0 - dot_val
    return (pos_diff < pos_threshold) and (ori_diff < ori_threshold)


def track_task_progress(articulation_controller, controller, target_pos, target_ori, task_state):
    if task_state.get("done", False):
        return

    # Retrieve end-effector pose from the custom helper
    current_pos, current_ori = get_end_effector_pose("/World/UR10/ee_link")
    if current_pos is None:
        print("Could not get EE pose, skipping this frame.")
        return

    # Check thresholds
    if is_close_enough(current_pos, target_pos, current_ori, target_ori):
        task_state["done"] = True
        print("Task completed!")
    else:
        # Not done; compute next action
        actions = controller.forward(
            target_end_effector_position=target_pos,
            target_end_effector_orientation=target_ori,
        )
        articulation_controller.apply_action(actions)

    print("Current EE Position:", current_pos)
    print("Target Position:", target_pos)
    print("Computed Actions:", actions)


# camera_instance.save_camera_frames()
socket = initialize_publisher()

cmd_socket = initialize_command_receiver()


def main() -> None:
    # Main simulation loop.
    external_control_active = False  # Flag to track external control
    reset_needed = False

    # Example: define a task
    target_position = np.array([0.5, 0.5, 0.3])
    target_orientation = np.array([0, 0, 0, 1])  # quaternion
    task_state = {"done": False}  # Track if task is complete

    while simulation_app.is_running():
        # Step the simulation with rendering enabled.
        world.step(render=True)

        # # Process external commands if available
        # if cmd_socket.poll(timeout=0):
        #     try:
        #         msg = cmd_socket.recv(zmq.NOBLOCK)
        #         command = json.loads(msg.decode("utf-8"))
        #         print("Received command:", command)
        #         if command.get("command") == "move":
        #             external_control_active = True
        #             target_position = np.array(command["target_position"])
        #             target_orientation = np.array(command["target_orientation"])
        #             # Debug: Print the target values
        #             print("Target Position:", target_position)
        #             print("Target Orientation:", target_orientation)
        #             # Compute the movement action
        #             actions = my_controller_RMP.forward(
        #                 target_end_effector_position=target_position,
        #                 target_end_effector_orientation=target_orientation,
        #             )
        #             # Debug: Print computed actions
        #             print("Computed Actions:", actions)
        #             articulation_controller.apply_action(actions)
        #     except Exception as e:
        #         print("Error processing command:", e)

        # ---------------------------------------------------------------------------------------------------------------------------------
        # Check if simulation is stopped to mark for reset.
        if world.is_stopped() and not reset_needed:
            reset_needed = True

        if world.is_playing():
            camera_instance.publish_camera_frames(socket)
            # If a reset is needed, reset the simulation and the robot controller.
            # if reset_needed:
            #     world.reset()
            #     my_controller_RMP.reset()
            #     reset_needed = False

            # If we still have a task in progress, keep moving the robot
            if not task_state["done"]:
                track_task_progress(articulation_controller, my_controller_RMP, target_position, target_orientation, task_state)

            else:
                # If we’re done, we could do something else or remain idle
                pass

            # # Only call default task if no external command is active.
            # if not external_control_active:
            #     # robot_look_at_table(articulation_controller, my_controller_RMP, my_controller_PP)
            #     print("Executing default task...")
            # else:
            #     # Optionally reset external control after a few steps or based on a condition.
            #     # For instance, reset the flag after applying the command for a few iterations.
            #     external_control_active = False

            # observations = world.get_observations()

    simulation_app.close()


if __name__ == "__main__":
    main()
