"""
File Description: A script(Python or Bash) that verifies the host machine is correctly configued for ROS2 before 
                   any nodes are launched. This catches "it works on my machine" erros early before deployment'

Language Decision: Python is much more reliable and self-contained than Bash because of the use of rclpy
                    (one process and deterministic timeout handling)

"""
import sys      # Allows us to exit the script with an error code if checks fail
import os       # A module/toolbox: allows to see what variabes are currently set in the shell 
import rclpy    # Interfaces with ROS2 middleware to create nodes

# --------------------------------------------------------------------------------------------------------------
# CHECK 1: VERIFY ROS IS SOURCED (ROS_DISTRO and AMENT_PREFIX_PATH)
# --------------------------------------------------------------------------------------------------------------
# Check the os module and fetch from the environ dictionary to retrive the active ROS version "ROS_DISTRO".
# The .get() method safely looks for the key. If missing, it returns None.
ros_distro = os.environ.get("ROS_DISTRO")  
         
# Check the os.environ dictionary to retrive the path where ROS is located
ament_prefix_path = os.environ.get("AMENT_PREFIX_PATH")

# Check the "Missing" Case.
if ros_distro is None: # Python uses is instead of ==, but both works
    print("The 'ROS_DISTRO' was not found")
    sys.exit(1)

if ament_prefix_path is None:
    print("The 'AMENT PREFIX PATH' was not found. Did you forget to run: source /opt/ros/humble/setup.bash ")
    sys.exit(1)

# If we get here, ROS is found. 
print (f"Sucess! ROS Distro '{ros_distro} is active")

# --------------------------------------------------------------------------------------------------------------
# CHECK 2: NETWORK CONFIGURATION (ROS_DOMAIN_ID)
# --------------------------------------------------------------------------------------------------------------
# Check for The ROS_DOMAIN_ID -> if not found, default to "0"
# We use "0" as a backup value in .get() because enviroments variables are always strings. 

ros_domain_id = os.environ.get("ROS_DOMAIN_ID","0") # Everything inside the dictionary is stored as a String
print(f"The ROS DOMAIN ID is: {ros_domain_id}")     # The 'f' tells Python to swap the {variable} inside the braces for its actual value

# --------------------------------------------------------------------------------------------------------------
# CHECK 3: ROS MIDDLEWARE
# --------------------------------------------------------------------------------------------------------------