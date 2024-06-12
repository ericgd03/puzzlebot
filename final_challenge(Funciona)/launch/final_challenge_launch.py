from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    path_node = Node(
        package = 'final_challenge',
        executable = 'path',
        output = 'screen',
    )
    
    controller_node = Node(
        package = 'final_challenge',
        executable = 'controller',
        output = 'screen',
    )

    odometry_node = Node(
        package = 'final_challenge',
        executable = 'odometry',
        output = 'screen',
    )

    l_d = LaunchDescription([odometry_node, controller_node, path_node])
    return l_d