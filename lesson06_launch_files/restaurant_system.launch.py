from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test2_py_pkg',
            executable='my_first_node',
            name='my_first_node',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='waiter',
            name='waiter',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='chef',
            name='chef',
            output='screen',
        ),
        # MUST BE HERE:
        Node(
            package='test2_py_pkg',
            executable='restaurant_server',
            name='restaurant_server',
            output='screen',
        ),
        Node(
            package='test2_py_pkg',
            executable='restaurant_client',
            name='restaurant_client',
            output='screen',
        ),
    ])
