import sys  # Allows us to exit the script with an error code if checks fail
import os  # A module/toolbox: allows to see what variables are currently set in the shell
import subprocess
from pathlib import Path

# Run a command and capture output
ws_dir = Path(__file__).resolve().parent.parent

try:
    result = subprocess.run(["source", "/opt/ros/jazzy/setup.bash"], check=True, cwd=ws_dir)
except subprocess.CalledProcessError as e:
    print("ROS2 Jazzy could not be sourced.\n\n")
    print(f"Command failed with return code: {e.returncode}")
    print(f"Command: {e.cmd}")
    print(f"Error output: {e.output}")

# --------------------------------------------------------------------------------------------------------------
# CHECK 1: VERIFY ROS IS SOURCED (ROS_DISTRO and AMENT_PREFIX_PATH)
# --------------------------------------------------------------------------------------------------------------
# Check the os module and fetch from the environ dictionary to retrieve the active ROS version "ROS_DISTRO".
# The .get() method safely looks for the key. If missing, it returns None.
ros_distro = os.environ.get("ROS_DISTRO")

# Check the os.environ dictionary to retrieve the path where ROS is located
ament_prefix_path = os.environ.get("AMENT_PREFIX_PATH")

# Check the "Missing" Case.
if ros_distro is None:  # Python uses is instead of ==, but both works
    print(
        "The 'ROS_DISTRO' was not found.  Did you forget to run: source /opt/ros/humble/setup.bash"
    )
    sys.exit(1)  # Exit the program with a status 1 to signal error/failure

if ament_prefix_path is None:
    print(
        "The 'AMENT_PREFIX_PATH' was not found. Did you forget to run: source /opt/ros/humble/setup.bash "
    )
    sys.exit(1)  # Exit the program with a status 1 to signal error/failure

# If we get here, ROS is found.
print(f"\nSucess! ROS Distro '{ros_distro}' is active")


# Try to build the ws
try:
    result = subprocess.run(["colcon", "build", "--symlink-install"], check=True, cwd=ws_dir)
except subprocess.CalledProcessError as e:
    print("workspace could not be built.\n\n")
    print(f"Command failed with return code: {e.returncode}")
    print(f"Command: {e.cmd}")
    print(f"Error output: {e.output}")
