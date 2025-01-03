from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    controller = LaunchConfiguration('controller')

    controller = DeclareLaunchArgument(
        'controller',
         default_value=''
    )
    param = {"cmd_name": "activate", "cmd_params":[controller]}
    active_controller = ExecuteProcess(
        cmd=[[
            'ros2 service call ',
            'control_node/control_command ',
            'control_msgs/srv/ControlCommand ',
            '"{cmd_name: activate, cmd_params: [RealtimeTestController]}"'
        ]],
        shell=True
    )
    

    return LaunchDescription([
        active_controller,
    ])