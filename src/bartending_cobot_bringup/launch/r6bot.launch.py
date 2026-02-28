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
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix


def generate_launch_description():
    # launch config vars to tell launch file that these args will be declared
    ur_type = LaunchConfiguration("ur_type")
    world_file = LaunchConfiguration("world_file")

    ur_description_path_share = FindPackageShare("ur_description")
    ur_description_path_prefix = FindPackagePrefix("ur_description")

    ur_description_launch_path = PathJoinSubstitution(
        [ur_description_path_share, "launch", "view_ur.launch.py"]
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    gz_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description"],
    )

    joint_state_broadcaster_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    r6bot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["r6bot_controller", "--controller-manager", "/controller_manager"],
    )

    # launch_ur_description = IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #         ur_description_launch_path
    #     ),
    #     launch_arguments={"ur_type": "ur5e"}.items(),
    # )

    robot_description = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
        arguments=[
            "/home/prem/code/bartending_cobot/src/bartending_cobot_description/urdf/bartending_cobot.xacro.urdf"
        ],
    )
    ld = LaunchDescription(
        [
            # Launch gazebo environment
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                        )
                    ]
                ),
                launch_arguments=[("gz_args", [" -r -v 1 empty.sdf"])],
            ),
            # make sure ur_description launch started and then robot is spawned in gazebo
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=robot_description,
                    on_start=[gz_spawn_robot],
                )
            ),
            # make sure robot is spawned and then joint_state_broadcaster is started
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_robot,
                    on_exit=[joint_state_broadcaster_controller_spawner],
                )
            ),
            # make sure r6bot controller started after joint_state_broadcaster controller
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_controller_spawner,
                    on_exit=[r6bot_controller_spawner],
                )
            ),
            bridge,
            robot_description,
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
            SetEnvironmentVariable(
                "GZ_SIM_PLUGIN_PATH",
                "/opt/ros/jazzy/lib/libgz_ros2_control-system.so",  # PathJoinSubstitution(["/opt","ros","jazzy","lib/"])
            ),
            SetEnvironmentVariable(
                "GAZEBO_MODEL_PATH", PathJoinSubstitution([ur_description_path_prefix, "share"])
            ),
        ]
    )

    return ld
