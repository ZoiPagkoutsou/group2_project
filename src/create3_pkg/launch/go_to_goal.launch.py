"""
Drive the Create 3 (robot_8) from where it starts (A) to a goal (B).

B is given before the start, relative to the start pose:
    ros2 launch create3_pkg go_to_goal.launch.py goal_x:=2.0 goal_y:=1.0
    -> 2 m forward, 1 m left. Negative goal_y = right, negative goal_x = back.

The wall follower node runs DistBug: it drives straight at the goal, goes
round any obstacle in the way (choosing the shorter side with the LiDAR when
auto_side is true), and stops when it reaches the goal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        # No default: the goal must be given at launch.
        DeclareLaunchArgument(
            'goal_x', description='Goal: metres forward of the start pose'),
        DeclareLaunchArgument(
            'goal_y', description='Goal: metres left of the start pose '
                                  '(negative = right)'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.10',
            description='Stop when this close to the goal (m)'),
        DeclareLaunchArgument(
            'auto_side', default_value='true',
            description='true = choose the side for each obstacle from the '
                        'LiDAR; false = always use follow_side'),
        DeclareLaunchArgument(
            'follow_side', default_value='right',
            description='Side used to go round obstacles when auto_side is '
                        "false (fallback otherwise): 'right' or 'left'"),
    ]

    def as_float(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    wall_follower = Node(
        package='create3_pkg',
        executable='wall_follower_node',
        name='wall_follower',
        namespace='robot_8',
        output='screen',
        parameters=[{
            'drive': True,
            'use_goal': True,
            'goal_x': as_float('goal_x'),
            'goal_y': as_float('goal_y'),
            'goal_tolerance': as_float('goal_tolerance'),
            'auto_side': ParameterValue(LaunchConfiguration('auto_side'),
                                        value_type=bool),
            'follow_side': LaunchConfiguration('follow_side'),
        }],
    )

    return LaunchDescription(arguments + [wall_follower])
