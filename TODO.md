# TODO

- [x] update README [usage section](README.md#usage).
- [ ] Implement environment tests to ensure that the development environment is correctly configured.
  - [ ] Gazebo Harmonic: Is the simulator available?
  - [ ] ros2_control: Are the necessary controllers present?
- [ ] Develop CI/CD pipelines for automated testing and deployment.
  - [x] `colcon build` test
  - [ ] Unit tests for each package
  - [ ] Integration tests for the overall system
  - [x] code quality checks using tools like `ament_lint`, `clang`, or `pylint`
- [x] Implement a package for simulation in Gazebo with a simple manipulator model.
    - [x] Configure ros2_control for the manipulator.
