# Copyright 2019 Louise Poubel
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

"""Launch Gazebo with a world that has Dolly, as well as the follow node."""

from os import environ, path

from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def update_environment_variable(name, value):
    print("Updating environment variable {}".format(name))
    # Get environment variable
    # Use empty if not found
    env_var = environ.get(str(name), '')
    # Append new value
    environ[str(name)] = env_var + ':' + str(value)
    print("> {}={}".format(str(name), env_var + ':' + str(value)))

def generate_launch_description():

    update_environment_variable(
        'GAZEBO_RESOURCE_PATH',
        '/colcon_ws/src/dolly/dolly_gazebo/worlds'
    )
    update_environment_variable(
        'GAZEBO_MODEL_PATH',
        '/colcon_ws/src/dolly/dolly_gazebo/models'
    )

    gzserver_exe = launch.actions.ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             launch.substitutions.LaunchConfiguration('world')],
        output='screen'
    )
    gzclient_exe = launch.actions.ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    follow = launch_ros.actions.Node(
        package='dolly_follow',
        node_executable='dolly_follow',
        output='screen',
        remappings=[
            ('cmd_vel', '/dolly/cmd_vel'),
            ('laser_scan', '/dolly/laser_scan')
        ]
    )
    go_to_goal = launch_ros.actions.Node(
        package='dolly_follow',
        node_executable='dolly_go_to_goal',
        output='screen',
        remappings=[
            ('cmd_vel', '/dolly/cmd_vel'),
        ]
    )

    rviz_config_dir = path.join(get_package_share_directory(
        'dolly_gazebo'), 'rviz', 'model.rviz')
    rviz2 = launch_ros.actions.Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=['worlds/empty.world', ''],
          description='Gazebo world file'),
        gzserver_exe,
        gzclient_exe,
        # follow,
        go_to_goal,
        rviz2,
    ])
