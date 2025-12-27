# ROS2-manipulator-viz-tool
This is a simple tool for visualizing the data published from a manipulator in an intuitive way, based on the principles illustrates in Tufte's book Envisioning Information.

The tool will visualize Dijkstra's algorithm in an intuitive way for showing how motion planning algorithms work. The tool will visualize the manipulator, obstacles, and a cost-heat map to provide insights on how the open and closed lists were calculated. The tool will also visualize the final path taken by the manipulator with a "lightning bolt" effect to highlight the final path. 

## Dev Setup
The following pre-requisites are necessary to run and develop the visualization tool:
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
    - Ensure `colcon` is installed and set up.
    - Ensure rviz2 is installed: `sudo apt install ros-jazzy-rviz2`
    - Ensure dependencies for the visualization tool are available:
        - pluginlib
        - rviz_common
        - rviz_rendering
        - tf2
- Install xacro if not already installed: `sudo apt install ros-jazzy-xacro`
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install/)
    - Ensure Gazebo is properly installed and configured to work with ROS 2 Jazzy. [Dependency install guide](https://gazebosim.org/docs/harmonic/ros_installation/): `sudo apt-get install ros-${ROS_DISTRO}-ros-gz`
- [ros2_control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)
    - Ensure ros2_control-gazebo communication is set up. [ros2_control installation guide](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)

## Development Milestones
1. Implement environment tests to ensure that the development environment is correctly configured.
    - ROS 2 Jazzy: Is it installed and sourced?
    - Gazebo Harmonic: Is the simulator available?
    - ros2_control: Are the necessary controllers present?
2. Develop CI/CD pipelines for automated testing and deployment.
    - `colcon build` test
    - Unit tests for each package
    - Integration tests for the overall system
    - code quality checks using tools like `ament_lint`, `clang`, or `pylint`
3. Implement a package for simulation in Gazebo with a simple manipulator model.
    - Configure ros2_control for the manipulator.
4. Create a package for the visualization tool using rqt or rviz2.
5. Create a package for motion planning using Dijkstra's algorithm.
