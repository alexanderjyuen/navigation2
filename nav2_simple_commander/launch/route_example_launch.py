# Copyright (c) 2025 Open Navigation LLC
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

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Define local map types
MAP_POSES_DICT = {
    'depot': {
        'x': -8.00, 'y': 0.00, 'z': 0.01,
        'R': 0.00, 'P': 0.00, 'Y': 0.00
    },
    'warehouse': {
        'x': 2.00, 'y': -19.65, 'z': 0.01,
        'R': 0.00, 'P': 0.00, 'Y': 0.00
    }
}

ROUTE_POSES_DICT = {
    'start': {
        'depot': {
            'x': 7.5, 'y': 7.5, 'yaw': 0.00  # 3rd node
        },
        'warehouse': {
            'x': 2.00, 'y': -19.65, 'yaw': 0.00  # 0th node
        }
    },
    'goal': {
        'depot': {
            'x': 20.12, 'y': 11.83, 'yaw': 0.00  # 13th node
        },
        'warehouse': {
            'x': -13.5, 'y': -3.15, 'yaw': 0.00  # 61st node
        }
    }
}
MAP_TYPE = 'depot'  # Change this to 'warehouse' for warehouse map


def generate_launch_description() -> LaunchDescription:
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')

    robot_sdf = os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro')
    world = os.path.join(sim_dir, 'worlds', f'{MAP_TYPE}.sdf')
    map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', f'{MAP_TYPE}.yaml')
    graph_filepath = os.path.join(
        nav2_bringup_dir, 'graphs', f'{MAP_TYPE}_graph.geojson')

    # Launch configuration variables
    headless = LaunchConfiguration('headless')
    use_keepout_zones = LaunchConfiguration('use_keepout_zones')
    use_speed_zones = LaunchConfiguration('use_speed_zones')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    speed_mask_yaml_file = LaunchConfiguration('speed_mask')

    # Declare the launch arguments
    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='False', description='Whether to execute gzclient)'
    )
    declare_use_keepout_zones_cmd = DeclareLaunchArgument(
        'use_keepout_zones', default_value='True',
        description='Whether to enable keepout zones or not'
    )

    declare_use_speed_zones_cmd = DeclareLaunchArgument(
        'use_speed_zones', default_value='True',
        description='Whether to enable speed zones or not'
    )
    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask',
        default_value=os.path.join(nav2_bringup_dir, 'maps', f'{MAP_TYPE}_keepout.yaml'),
        description='Full path to keepout mask file to load',
    )
    declare_speed_mask_yaml_cmd = DeclareLaunchArgument(
        'speed_mask',
        default_value=os.path.join(nav2_bringup_dir, 'maps', f'{MAP_TYPE}_speed.yaml'),
        description='Full path to speed mask file to load',
    )

    # start the simulation
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world_sdf]}.items())

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(sim_dir, 'worlds'))
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        condition=IfCondition(PythonExpression(
            ['not ', headless])),
        launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb4.launch.py')),
        launch_arguments={'use_sim_time': 'True',
                          'robot_sdf': robot_sdf,
                          'x_pose': str(MAP_POSES_DICT[MAP_TYPE]['x']),
                          'y_pose': str(MAP_POSES_DICT[MAP_TYPE]['y']),
                          'z_pose': str(MAP_POSES_DICT[MAP_TYPE]['z']),
                          'roll': str(MAP_POSES_DICT[MAP_TYPE]['R']),
                          'pitch': str(MAP_POSES_DICT[MAP_TYPE]['P']),
                          'yaw': str(MAP_POSES_DICT[MAP_TYPE]['Y']),
                          }.items())

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True, 'robot_description': Command(['xacro', ' ', robot_sdf])}
        ]
    )

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('map', map_yaml_file),
            ('graph', graph_filepath),
            ('use_keepout_zones', use_keepout_zones),
            ('use_speed_zones', use_speed_zones),
            ('keepout_mask', keepout_mask_yaml_file),
            ('speed_mask', speed_mask_yaml_file),
        ])

    # start the demo autonomy task
    demo_cmd = Node(
        package='nav2_simple_commander',
        executable='example_route',
        parameters=[{
            'start_pose': {
                'x': ROUTE_POSES_DICT['start'][MAP_TYPE]['x'],
                'y': ROUTE_POSES_DICT['start'][MAP_TYPE]['y'],
                'yaw': ROUTE_POSES_DICT['start'][MAP_TYPE]['yaw']
            },
            'goal_pose': {
                'x': ROUTE_POSES_DICT['goal'][MAP_TYPE]['x'],
                'y': ROUTE_POSES_DICT['goal'][MAP_TYPE]['y'],
                'yaw': ROUTE_POSES_DICT['goal'][MAP_TYPE]['yaw']
            },
        }],
        emulate_tty=True,
        output='screen')

    set_env_vars_resources2 = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            str(Path(os.path.join(desc_dir)).parent.resolve()))

    ld = LaunchDescription()
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_use_speed_zones_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_speed_mask_yaml_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(world_sdf_xacro)
    ld.add_action(remove_temp_sdf_file)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(demo_cmd)
    ld.add_action(set_env_vars_resources2)
    return ld
