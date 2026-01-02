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

### Github Actions
This repo implements various github actions to ensure a smooth CI/CD process and enable faster prototyping. The actions implemented are described below:
- [pylint.yaml](.github/workflows/pylint.yml) workflow: This action is designed to ensure that code is the highest quality. It will find all the python files in the repo and then produce a lint.txt as an artifact which can be downloaded from Github.
- [colcon_build.yaml](.github/workflows/colcon_build.yml) workflow: This action will create a ros2_jazzy dev env, install any dependencies, and then build the workspace. This action ensures that the committed code can be build on another machine. Currently, the tests are skipped since the codebase is fairly new. This will be removed once the codebase develops. More details available [here](https://github.com/ros-tooling/action-ros-ci).

### How to use Git for this project:
The project uses Git for version control. In order to ensure a clean, organized commit history, please follow these guidelines:
- Make sure `pre-commit` command is installed.
    - VS Code may return an error when using the IDE for committing changes. Try using the CLI `git commit -m "<message>"` if these errors cause issues.
- Create a new branch for each feature or bug fix you work on. Use descriptive names for your branches.
- Write clear and concise commit messages that describe the changes you made.
- Before pushing your changes, ensure that your code is properly formatted and passes all tests.
- Regularly pull changes from the main branch to keep your branch up to date.
- Ensure that minor changes are squashed into larger commits to keep the commit history clean.
    - Use interactive rebase (`git rebase -i`) to squash commits before merging ([guide](https://thoughtbot.com/blog/git-interactive-rebase-squash-amend-rewriting-history)).

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


## TODO
- Once the codebase is mature enough, remove the `skip-tests: true` line from the [colcon_build.yaml](.github/workflows/colcon_build.yml) workflow.
