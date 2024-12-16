# Copyright 2024 Alessandro Sofia, Herobots
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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import TimerAction

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='robot_drive.config.xacro',
            description='URDF/XACRO description file with the axis.',
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'num_drives',
            default_value='3',
            description='Number of drives.',
        )
    )

    description_file = LaunchConfiguration('description_file')
    num_drives = LaunchConfiguration('num_drives')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ethercat_zeroerr"),
                    "description/config",
                    description_file,
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ethercat_zeroerr"),
            "config",
            "robot_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "-c", "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "-c", "/controller_manager"],
    )

    delayed_trajectory_spawner = TimerAction(
        period=PythonExpression(["5.0 * ", num_drives]),
        actions=[trajectory_controller_spawner]
    )
    
    delayed_velocity_spawner = TimerAction(
        period=PythonExpression(["5.0 * ", num_drives]),
        actions=[velocity_controller_spawner]
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delayed_trajectory_spawner
        # delayed_velocity_spawner
    ]

    return LaunchDescription(
        declared_arguments +
        nodes)
