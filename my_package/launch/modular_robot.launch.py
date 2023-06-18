# my_package/launch/traj_action.launch.py
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'test.yaml'
        )
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "modular_robot", 
            package='my_package', 
            executable='module_robot', 
            parameters=[config],
            output='screen'),
        # launch_ros.actions.Node(
        #     namespace= "turtlesim2", package='my_package', executable='turtlesim_node', output='screen'),
    ])