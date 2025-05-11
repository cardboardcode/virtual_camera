from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, **kwargs):
    use_image_viewer = LaunchConfiguration('use_image_viewer').perform(context)
    nodes_to_launch = []

    nodes_to_launch.append(
        Node(
            package = 'virtual_camera',
            executable = 'image_publisher',
            output = 'screen',
            emulate_tty= True
        )
    )

    if use_image_viewer == 'True':
        nodes_to_launch.append(
            Node(
                package = 'virtual_camera',
                executable = 'image_subscriber',
                output = 'screen',
                emulate_tty= True
            )
        )

    return nodes_to_launch

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_image_viewer',
            default_value='True',
            description='Set use_image_viewer [yes/no]'
        ),
        OpaqueFunction(function=launch_setup)
    ])