# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

# To run:
# sudo -E bash src/pi3hat_hardware_interface/run_as_root.sh ros2 launch src/pi3hat_hardware_interface/bringup/launch/test_joint_controller_debug1.launch.py


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start.",
        )
    )

    robot_controller = LaunchConfiguration("robot_controller")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("pi3hat_hardware_interface"),
                    "urdf",
                    "test_state_publisher.urdf.xacro",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pi3hat_hardware_interface"),
            "config",
            "test_joint_controller.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        prefix=['valgrind --tool=callgrind --dump-instr=yes -v --collect-systime=yes --instr-atstart=no --vgdb=yes'],
        # prefix=['valgrind --tool=helgrind -v --vgdb=yes'],
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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    nodes = [
        control_node,
        robot_state_pub_node,
    ]

    return LaunchDescription(
        # declared_arguments
        nodes
    )