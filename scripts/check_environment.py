"""
File Description: A script(Python or Bash) that verifies the host machine is correctly configued for ROS2 before 
                   any nodes are launched. This catches "it works on my machine" erros early before deployment

Language Decision: Python is much more reliable and self-contained than Bash because of the use of rclpy
                    (one process and deterministic timeout handling)

"""
import sys      # Allows us to exit the script with an error code if checks fail
import os       # A module/toolbox: allows to see what variabes are currently set in the shell 
import time     # Provides timing utilities we need for deterministic timeouts in the loopback test

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
    print("The 'ROS_DISTRO' was not found.  Did you forget to run: source /opt/ros/humble/setup.bash")
    sys.exit(1) # Exit the program with a status 1 to signal error/failure

if ament_prefix_path is None:
    print("The 'AMENT_PREFIX_PATH' was not found. Did you forget to run: source /opt/ros/humble/setup.bash ")
    sys.exit(1)  # Exit the program with a status 1 to signal error/failure

# If we get here, ROS is found. 
print (f"\nSucess! ROS Distro '{ros_distro}' is active")

# --------------------------------------------------------------------------------------------------------------
# CHECK 2: NETWORK CONFIGURATION (ROS_DOMAIN_ID)
# --------------------------------------------------------------------------------------------------------------
try:  # Wrap ROS imports so we can exit with a clear error message instead of a stack trace
    import rclpy    # Interfaces with ROS2 middleware to create nodes
    from std_msgs.msg import String  # Imports a simple ROS 2 message type used for our loopback test
except ImportError as exc:  # If ROS Python packages are missing, we handle it gracefully
    print("ERROR: Could not import rclpy/std_msgs. Did you source ROS 2 (source /opt/ros/humble/setup.bash)?")  # Explain the likely root cause in simple terms
    print(f"ImportError details: {exc}")  # Print the underlying import error to help debugging
    sys.exit(2)  # Exit with a unique non-zero code to signal a ROS Python dependency/import problem

# Check for The ROS_DOMAIN_ID -> if not found, default to "0"
# We use "0" as a backup value in .get() because enviroments variables are always strings. 
ros_domain_id = os.environ.get("ROS_DOMAIN_ID","0") # Everything inside the dictionary is stored as a String
print(f"The ROS DOMAIN ID is: {ros_domain_id}")     # The 'f' tells Python to swap the {variable} inside the braces for its actual value

# 1. Initialize the ROS2 rclpy library = init()
# Passing sys.argv allows the script to handle ROS-specific command-line arguments (e.g remapping)
rclpy.init(args=sys.argv) 

# 2. Create a Node (i.e. test_node)
node = rclpy.create_node("test_node")

# 3. Create the Publisher (i.e. the Speaker)
message_type = String # imported
queue_size = 10       # safe default
topic_name = "test_ping"
publisher = node.create_publisher(message_type, topic_name, queue_size)     # type: ignore # giving our node the ability to speak

# 4. Create the Subscriber (i.e. the Ear)
received_list: list[str] = []                  # creates an empty list
def hear_message(msg: String):              # in python functions are always define above
    received_list.append(msg.data)  # type: ignore # adds to the list
    print(f"Received: {msg.data}")   # type: ignore

# hear_message: is the name of the function and is use later when a message arrives.
# since the Subscriber needs to save the function, we do not use parentheses here
subscriber = node.create_subscription(message_type, topic_name, hear_message, queue_size) # type: ignore # giving our node the ability to hear

# 5. Publish a message (i.e. Place it inside a message object)
msg = String()         # create a blank String message object (i.e. The Envelope) 
msg.data = "Hello ROS" # put the text into the 'data' slot (i.e The Letter)
publisher.publish(msg) # send the entire object msg over the network (i.e. Drop it in the Mailbox)

# 6. This gives the message time to travel out and come back
rclpy.spin_once(node, timeout_sec =1) # spin only once to check for replies

start_time = time.time()      # Record the start time so we can enforce a hard timeout for this test
timeout_sec = 5.0             # Maximum amount of time we will wait for the subscriber to hear the message
republish_interval_sec = 0.5  # How often we re-publish the test message during the waiting window
last_publish_time = start_time# Track when we last published so we can re-publish periodically

# Keep spinning until we receive or we time out
while (len(received_list) == 0) and ((time.time() - start_time) < timeout_sec):
    rclpy.spin_once(node, timeout_sec=0.1)  # Spin in small chunks to process DDS discovery and callbacks reliably
    # If enough time passed, publish again to avoid missing the first message
    if (time.time() - last_publish_time) >= republish_interval_sec:
        publisher.publish(msg)          # Re-send the same message object to increase reliability during discovery
        last_publish_time = time.time() # Update our publish timestamp after re-sending

# 7. Ensure the Subscriber heard the message
if len(received_list) == 0: # the length of the list is 0 the list is empty
    print("The Subscriber DID NOT hear the message")
    node.destroy_node()# Destroy the node explicitly so resources are cleaned up even on failure
    rclpy.shutdown()   # Ensure rclpy is shut down before exiting so the script exits cleanly
    sys.exit(3)        # Exit with a unique non-zero code to signal a DDS loopback timeout/failure
else:
    print("The Subscriber heard the message")

# 8. Shutdown
node.destroy_node()  # Destroy the node explicitly so we leave no ROS resources behind
rclpy.shutdown()     # clean up ROS before doing non-ROS checks

# --------------------------------------------------------------------------------------------------------------
# CHECK 3: ROBOT MIDDLEWARE (RMW)-hidden layer that handles all the message passing between nodes (robot postal service)
# --------------------------------------------------------------------------------------------------------------

rmw_implementation = os.environ.get("RMW_IMPLEMENTATION")  # Read the requested RMW from the environment (None means "use the default")
known_rmw = {"rmw_fastrtps_cpp", "rmw_cyclonedds_cpp", "rmw_connextdds"}  # Define a small known-good set of common ROS 2 RMW implementations

if rmw_implementation is None:  # If unset, ROS 2 will use the system default RMW implementation
    print("The RMW Implementation is not set (ROS 2 will use its default)")
else:
    if rmw_implementation not in known_rmw:                                # If the value is unexpected, this could indicate a misconfiguration
        print(f"Error: Unknown RMW_IMPLEMENTATION='{rmw_implementation}'") # Explain that the env var value is not recognized
        print(f"Known values include: {sorted(known_rmw)}")  # Show the user acceptable values to reduce guesswork
        sys.exit(4)                                          # Exit with a unique non-zero code to signal middleware configuration inconsistency
    print(f"The RMW Implementation is: {rmw_implementation}")# Confirm the middleware choice when it is explicitly set and recognized


# CHECKS COMPLETED -----------------------------------
print("We are done here, and everything looks good\n")
sys.exit(0)  # Exit with 0 to indicate overall success when all checks pass



