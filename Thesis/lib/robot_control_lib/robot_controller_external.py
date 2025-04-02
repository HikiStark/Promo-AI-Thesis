import zmq
import json
import time


def main():
    context = zmq.Context()
    # Use a PUSH socket to send commands to the simulation's PULL socket.
    socket = context.socket(zmq.PUSH)
    # Connect to the simulation (assuming it's running on localhost and port 5560)
    socket.connect("tcp://localhost:5560")

    # Give the connection a moment to establish.
    time.sleep(1)

    # Create a command message with a target position and orientation.
    command = {
        "command": "move",
        "target_position": [0.5, 0.5, 0.3],
        # New target: use the robot’s natural EE orientation (close to measured value)
        "target_orientation": [1, 0, 0, 0],  # New target: use the robot’s natural EE orientation (close to measured value)
        # "target_orientation": [0, 0, 0, 1] # Old target (identity)
    }  # Example target position  # Example target orientation (quaternion)

    # Send the command as a JSON string.
    socket.send(json.dumps(command).encode("utf-8"))
    print("Sent command:", command)

    # Optionally, you can send more commands or loop to send continuous updates.
    # time.sleep(1)


if __name__ == "__main__":
    main()
