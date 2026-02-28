# Copyright 2023 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix


def generate_launch_description():
    # launch config vars to tell launch file that these args will be declared
    ur_type = LaunchConfiguration("ur_type")
    world_file = LaunchConfiguration("world_file")

    ros_gz_sim_pkg_path = FindPackageShare("ros_gz_sim")
    ur_description_path_share = FindPackageShare("ur_description")
    ur_description_path_prefix = FindPackagePrefix("ur_description")

    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, "launch", "gz_sim.launch.py"])

    ur_description_launch_path = PathJoinSubstitution(
        [ur_description_path_share, "launch", "view_ur.launch.py"]
    )

    # launch_ur_description = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         ur_description_launch_path
    #     ),
    #     launch_arguments={"ur_type": "ur5e"}.items(),
    # )

    robot_description = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=["/home/prem/code/bartending_cobot/src/bartending_cobot_description/urdf/bartending_cobot.xacro.urdf"]
        )
            DeclareLaunchArgument(
                "ur_type",
                default_value="ur5e",
                description="type of Universal robot manipulator to load in",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="empty.sdf",
                description="path/name of SDF file to launch Gazebo with ",
            ),
            SetEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", PathJoinSubstitution([ur_description_path_prefix, "share"])
            ),
            # SetEnvironmentVariable(
            #     'GZ_SIM_PLUGIN_PATH',
            #     PathJoinSubstitution([ur_description_mesh_path])
            # ),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH", PathJoinSubstitution([ur_description_path_prefix, "share"])
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch_path),
                launch_arguments={
                    # Replace with your own world file
                    "gz_args": [world_file],
                    "on_exit_shutdown": "True",
                }.items(),  # type: ignore
            ),
            IncludeLaunchDescription(
                launch_description_source=PythonLaunchDescriptionSource(
                    ur_description_launch_path
                ),
                launch_arguments={"ur_type": ur_type}.items(),
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-topic", "robot_description"],
            ),
            # # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
            # Node(
            #     package='ros_gz_bridge',
            #     executable='parameter_bridge',
            #     arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',],
            #     remappings=[('/example_imu_topic',
            #                  '/remapped_imu_topic'),],
            #     output='screen'
            # ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
        ]
    )
