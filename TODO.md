# TODO

- [ ] update README [usage section](README.md#usage).
- [ ] Implement environment tests to ensure that the development environment is correctly configured.
  - [ ] Gazebo Harmonic: Is the simulator available?
  - [ ] ros2_control: Are the necessary controllers present?
- [ ] Develop CI/CD pipelines for automated testing and deployment.
  - [ ] `colcon build` test
  - [ ] Unit tests for each package
  - [ ] Integration tests for the overall system
  - [ ] code quality checks using tools like `ament_lint`, `clang`, or `pylint`
- [ ] Implement a package for simulation in Gazebo with a simple manipulator model.
    - [ ] Configure ros2_control for the manipulator.
