from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        # Start GoToPose Server with parameters
        Node(
            package='turtlesim_servers',
            executable='go_to_pose',
            name='go_to_pose_server',
            parameters=[{
                'max_linear_speed': 1.5,
                'max_angular_speed': 1.5
            }]
        ),
        # Start Reset Server
        Node(
            package='turtlesim_servers',
            executable='reset_server',
            name='reset_server'
        ),
        # Start Draw Square Server
        Node(
            package='turtlesim_servers',
            executable='draw_square',
            name='draw_square_server'
        )
    ])