# Copyright 2025 Bey Hao Yun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    imageviewer_flag_param = DeclareLaunchArgument(
        'use_image_viewer',
        default_value='false',
        description='Set use_image_viewer [true/false]'
    )

    use_debug_param = DeclareLaunchArgument(
        'use_debug',
        default_value='false',
        description='Enable debug logging output [true/false]'
    )

    # Create LaunchConfigurations
    use_debug = LaunchConfiguration('use_debug')

    # Build arguments dynamically at runtime
    # This approach is safe because launch will evaluate LaunchConfiguration later
    def make_node_arguments(context):
        if context.perform_substitution(use_debug) == 'true':
            return ['--ros-args', '--log-level', 'virtual_camera:=debug']
        else:
            return []

    from launch.actions import OpaqueFunction
    def make_vcam_node(context, *args, **kwargs):
        return [Node(
            package='virtual_camera',
            executable='virtual_camera',
            output='screen',
            parameters=[{
                'use_image_viewer': LaunchConfiguration('use_image_viewer')
            }],
            arguments=make_node_arguments(context)
        )]

    vcam_node_action = OpaqueFunction(function=make_vcam_node)

    return LaunchDescription([
        imageviewer_flag_param,
        use_debug_param,
        vcam_node_action
    ])

