# Copyright 2026 Bey Hao Yun
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    imageviewer_flag_param = DeclareLaunchArgument(
        'use_image_viewer',
        default_value='false',
        description='Attach in-process ImageViewer [true/false]'
    )

    use_debug_param = DeclareLaunchArgument(
        'use_debug',
        default_value='false',
        description='Enable debug logging output [true/false]'
    )

    fps_param = DeclareLaunchArgument(
        'fps',
        default_value='24',
        description='Publishing frame rate in FPS'
    )

    ws_url_param = DeclareLaunchArgument(
        'ws_url',
        default_value='ws://localhost:9090',
        description='Rosbridge WebSocket URL'
    )

    start_rosbridge_param = DeclareLaunchArgument(
        'start_rosbridge',
        default_value='true',
        description='Whether to automatically launch rosbridge_server [true/false]'
    )

    def launch_setup(context, *args, **kwargs):
        use_image_viewer = (
            LaunchConfiguration('use_image_viewer').perform(context).lower()
            == 'true'
        )
        use_debug = (
            LaunchConfiguration('use_debug').perform(context).lower()
            == 'true'
        )
        fps = LaunchConfiguration('fps').perform(context)
        ws_url = LaunchConfiguration('ws_url').perform(context)
        start_rosbridge = (
            LaunchConfiguration('start_rosbridge').perform(context).lower()
            == 'true'
        )

        actions = []

        if start_rosbridge:
            actions.append(
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('rosbridge_server'),
                            'launch',
                            'rosbridge_websocket_launch.xml'
                        ])
                    )
                )
            )

        cmd = ['virtual_camera', '--fps', fps, '--ws-url', ws_url]
        if use_image_viewer:
            cmd.append('--use-image-viewer')
        if use_debug:
            cmd.append('--debug')

        actions.append(ExecuteProcess(cmd=cmd, output='screen'))
        return actions

    return LaunchDescription([
        imageviewer_flag_param,
        use_debug_param,
        fps_param,
        ws_url_param,
        start_rosbridge_param,
        OpaqueFunction(function=launch_setup)
    ])
