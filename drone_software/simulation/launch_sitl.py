#!/usr/bin/env python3
import subprocess
import os

def start_px4_gazebo():
    # Path to PX4-Autopilot folder
    px4_path = os.path.expanduser("~/PX4-Autopilot")

    # Start simulation environment command
    # Starts a simulation with Quadrotor (x500)
    command = ["make", "px4_sitl", "gz_x500"]

    try:
        process = subprocess.Popen(
            command,
            cwd=px4_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True
        )

        # Stream output
        for line in process.stdout:
            print(line, end="")

        process.wait()

        if process.returncode == 0:
            print("Simulation started successfully.")
        else:
            print("Simulation fialed with return code:", process.returncode)
            print("Error output:")
            print(process.stderr.read())

    except Exception as e:
        print("Error occurred while starting PX4 Gazebo:", e)

__name__ == "__main__":
    start_px4_gazebo()
















