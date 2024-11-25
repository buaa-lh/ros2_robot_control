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

from launch.actions import RegisterEventHandler, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription



def generate_launch_description():

    # load_gripper_parameter_name = "load_gripper"
    # load_gripper = LaunchConfiguration(load_gripper_parameter_name)

    # ee_id_parameter_name = "ee_id"
    # ee_id = LaunchConfiguration(ee_id_parameter_name)

    # arm_id_parameter_name = "arm_id"
    # arm_id = LaunchConfiguration(arm_id_parameter_name)

    rviz_file = os.path.join(
        get_package_share_directory("franka_description"),
        "rviz",
        "visualize_franka.rviz",
    )

    
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        "fr3",
        "fr3" + ".urdf.xacro",
    )
    robot_description = xacro.process_file(
        franka_xacro_filepath, mappings={"hand": "false", "ee_id": "frank_hand"}
    ).toprettyxml(indent="  ")

    with open("urdf/fr3.urdf", "w") as f:
        f.write(robot_description)
        f.close()

    params = PathJoinSubstitution(
        [
            FindPackageShare("control_node"),
            "config",
            "control_params.yaml",
        ]
    )
  
    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        )
    
    rviz_node = Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["--display-config", rviz_file],
            )
    
    control_node = Node(
        package="control_node",
        executable="control_node",
        parameters=[params, {"robot_description": robot_description}],
        # arguments=["--ros-args", "--params-file", params],
        output="both",
    )

    arguments = [
            # DeclareLaunchArgument(
            #     load_gripper_parameter_name,
            #     default_value="false",
            #     description="Use end-effector if true. Default value is franka hand. "
            #     "Robot is loaded without end-effector otherwise",
            # ),

            # DeclareLaunchArgument(
            #     ee_id_parameter_name,
            #     default_value="franka_hand",
            #     description="ID of the type of end-effector used. Supporter values: "
            #     "none, franka_hand, cobot_pump",
            # ),
            # DeclareLaunchArgument(
            #     arm_id_parameter_name,
            #     default_value="fr3",
            #     description="ID of the type of arm used. Supporter values: "
            #     "fer, fr3, fp3",
            # )
            ]

    nodes = arguments + [
            rviz_node,
            robot_state_publisher,
            control_node,
            ]

    return LaunchDescription(nodes)
