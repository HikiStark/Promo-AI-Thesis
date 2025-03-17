# client.py: Run this on your external machine or as a separate process
import socket
import cv2
import numpy as np
import struct


def recvall(sock, count):
    """
    Helper function to receive exactly 'count' bytes from the socket.
    """
    buf = b""
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf


def main():
    host = "localhost"  # Change this to the IP address of the machine running the server if needed
    port = 8000

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    print("Connected to server")

    try:
        while True:
            # First, retrieve the length of the incoming image (4 bytes)
            length_buf = recvall(client_socket, 4)
            if length_buf is None:
                break
            (length,) = struct.unpack("!I", length_buf)

            # Now receive the image data based on the length
            data = recvall(client_socket, length)
            if data is None:
                break

            # Convert the byte data to a NumPy array and decode the image
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if frame is not None:
                cv2.imshow("Camera Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except Exception as e:
        print("Error:", e)
    finally:
        client_socket.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
