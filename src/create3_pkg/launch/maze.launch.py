"""
Launch the maze-solving system for the Create 3 (robot_8).

Full system (default):
    ros2 launch create3_pkg maze.launch.py
    wall_follower (perception only) -> navigation (Pledge) -> movement
    controller, with heading_publisher giving the heading.

Wall follower alone (it drives the robot itself with cmd_vel):
    ros2 launch create3_pkg maze.launch.py standalone:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    standalone = LaunchConfiguration('standalone')
    follow_side = LaunchConfiguration('follow_side')
    ir_threshold = LaunchConfiguration('ir_threshold')

    arguments = [
        DeclareLaunchArgument(
            'standalone', default_value='false',
            description='true = only the wall follower runs and drives the '
                        'robot; false = full Pledge navigation system'),
        DeclareLaunchArgument(
            'follow_side', default_value='right',
            description="Wall followed by the wall follower: 'right' or 'left'"),
        DeclareLaunchArgument(
            'ir_threshold', default_value='1000',
            description='Front IR value that blocks FORWARD in the movement '
                        'controller'),
    ]

    # Namespace robot_8 so ~/distances becomes
    # /robot_8/wall_follower/distances, the topic navigation listens to.
    # It drives (cmd_vel) only in standalone mode; in the full system the
    # movement controller is the only node that moves the robot.
    wall_follower = Node(
        package='create3_pkg',
        executable='wall_follower_node',
        name='wall_follower',
        namespace='robot_8',
        output='screen',
        parameters=[{
            'drive': ParameterValue(standalone, value_type=bool),
            'follow_side': follow_side,
        }],
    )

    # The heading publisher already uses 'robot_8/...' topic names, so it
    # must NOT get a namespace. The other two use absolute topic names.
    pledge_system = GroupAction(
        condition=UnlessCondition(standalone),
        actions=[
            Node(
                package='create3_pkg',
                executable='heading_publisher',
                output='screen',
            ),
            Node(
                package='create3_pkg',
                executable='navigation_maze_solving',
                output='screen',
            ),
            Node(
                package='create3_pkg',
                executable='movement_controller_new',
                output='screen',
                parameters=[{
                    'ir_threshold': ParameterValue(ir_threshold,
                                                   value_type=int),
                }],
            ),
        ],
    )

    return LaunchDescription(arguments + [wall_follower, pledge_system])
