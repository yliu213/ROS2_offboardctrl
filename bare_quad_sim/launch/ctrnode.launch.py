from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bare_quad_sim', 
            executable='single_quad',        
            name='single_quad',
            output='screen',
            parameters=[{
            }]
        )
    ])