# Created by Ofir Dvantman
# Function: Sending joint position to the robot using udp streaming

import socket
import struct
import time

def main():
    # Robot's IP and port
    robot_ip = "192.168.125.1"  # Replace with your robot's IP
    port = 6511  # EGM default port
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Example joint positions
    joint_positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    try:
        while True:
            # Pack joint positions into binary format
            message = struct.pack('6f', *joint_positions)

            # Send the message to the robot
            udp_socket.sendto(message, (robot_ip, port))
            print(f"Sent joint positions: {joint_positions}")

            # Sleep for 4ms to match the EGM sample rate
            time.sleep(0.004)

    except KeyboardInterrupt:
        print("UDP streaming stopped.")
    finally:
        udp_socket.close()

if __name__ == "__main__":
    main()

