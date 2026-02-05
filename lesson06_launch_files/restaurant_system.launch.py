from launch import LaunchDescription
# Main class for describing a launch scenario
from launch_ros.actions import Node
# Node action allows us to start ROS2 nodes


def generate_launch_description():
    # REQUIRED FUNCTION
    # ROS2 calls this function and executes what it returns
    
    return LaunchDescription([
        # List of everything to launch
        
        # NODE 1: Start the Restaurant Server
        # This must start first so clients can connect
        Node(
            package='test2_py_pkg',
            # Which package contains the executable
            
            executable='restaurant_server',
            # Name of the executable (from setup.py entry_points)
            
            name='restaurant_server_node',
            # Name this node will have in ROS2 (shows up in ros2 node list)
            
            output='screen'
            # 'screen' means output appears in your terminal
            # 'log' means output goes to a file
        ),
        
        # NODE 2: Start the Waiter (Publisher)
        Node(
            package='test2_py_pkg',
            executable='waiter',
            name='waiter_node',
            output='screen'
        ),
        
        # NODE 3: Start the Chef (Subscriber)
        Node(
            package='test2_py_pkg',
            executable='chef',
            name='chef_node',
            output='screen'
        ),
        
    ])
    # Return the LaunchDescription with all 3 nodes
