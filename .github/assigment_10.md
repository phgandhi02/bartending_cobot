# Create Environment and Network Diagnostic Script (Bash) #10

## Description: 
A script (Python or Bash) that verifies the host machine is correctly configured for ROS 2 before any nodes are launched. This catches "it works on my machine" errors early.

## Task List:

1) Create scripts/check_environment.py.

2) Check 1 (Sourcing): 
    - Verify ROS_DISTRO and AMENT_PREFIX_PATH environment variables are set.


3) Check 2 (Network): 
    - Run a quick loopback test (Create a temporary publisher/subscriber pair) to ensure the firewall isn't blocking DDS traffic.


4) Check 3 (Middleware): 
    - Verify RMW_IMPLEMENTATION is consistent (e.g., ensure you aren't mixing FastDDS and CycloneDDS accidentally).

5) Acceptance Criteria:
    - Running the script returns 0 (Success) on a configured machine.

    - Running the script returns non-zero + specific error message if source /opt/ros/humble/setup.bash was skipped.