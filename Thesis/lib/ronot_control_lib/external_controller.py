import zmq
import json
import time

# from lib.tasks.robot_look_table import robot_look_at_table


def robot_move_to_target(articulation_controller, target_position, target_orientation):
    # Execute the task where the robot looks at the table.
    # Example: Move to a specific target position
    target_position = np.array([0.5, 0.5, 0.3])  # Define specific target position
    target_orientation = np.array([0, 0, 0, 1])  # Define specific target orientation (quaternion)
    actions = my_controller_RMP.forward(
        target_end_effector_position=target_position,
        target_end_effector_orientation=target_orientation,
    )
    # Execute actions
    articulation_controller.apply_action(actions)


def main():
    context = zmq.Context()
    # Use a PUSH socket to send commands to the simulation's PULL socket.
    socket = context.socket(zmq.PUSH)
    # Connect to the simulation (assuming it's running on localhost and port 5560)
    socket.connect("tcp://localhost:5560")

    # Give the connection a moment to establish.
    time.sleep(1)

    # Create a command message with a target position and orientation.
    command = {"command": "move", "target_position": [2.5, 1.5, 0.3], "target_orientation": [0, 0, 0, 1]}  # Example target position  # Example target orientation (quaternion)

    # Send the command as a JSON string.
    socket.send(json.dumps(command).encode("utf-8"))
    print("Sent command:", command)

    # Optionally, you can send more commands or loop to send continuous updates.
    # time.sleep(1)


if __name__ == "__main__":
    main()
