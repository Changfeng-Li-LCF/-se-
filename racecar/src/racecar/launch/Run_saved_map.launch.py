"""Fixed occupancy map + AMCL; Cartographer supplies continuous local odometry."""
import os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('racecar')
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError('Map does not exist: ' + str(map_path))
    metadata = yaml.safe_load(map_path.read_text())
    image = Path(metadata['image'])
    if not image.is_absolute():
        image = map_path.parent / image
    if not image.is_file():
        raise RuntimeError('Map image does not exist: ' + str(image))
    sim = LaunchConfiguration('use_sim_time')
    managed = LaunchConfiguration('managed_startup').perform(context).lower() == 'true'
    autostart = not managed
    localization = os.path.join(share, 'config', 'saved_map_localization.yaml')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'Run_explorer.launch.py')),
            launch_arguments={
                'enable_navigation': 'false',
                'cartographer_config': 'racecar_local_odom.lua',
                'publish_map': 'false',
                'use_sim_time': sim,
                'no_rviz': LaunchConfiguration('no_rviz'),
                'rviz_config': LaunchConfiguration('rviz_config'),
            }.items()),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'use_sim_time': sim, 'yaml_filename': str(map_path)}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[localization, {'use_sim_time': sim}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': sim, 'autostart': autostart,
                          'node_names': ['map_server', 'amcl'], 'bond_timeout': 0.0}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'navigation_no_smoother.launch.py')),
            launch_arguments={'use_sim_time': sim,
                              'params_file': os.path.join(share, 'config', 'nav_carto.yaml'),
                              'use_composition': 'False', 'autostart': str(autostart).lower()}.items(),
            condition=UnlessCondition(LaunchConfiguration('plan_and_follow_only'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'navigation_follow_path.launch.py')),
            launch_arguments={'use_sim_time': sim, 'params_file': os.path.join(share, 'config', 'nav_carto.yaml'),
                              'with_planner': 'true', 'enable_through_poses': 'true',
                              'autostart': str(autostart).lower()}.items(),
            condition=IfCondition(LaunchConfiguration('plan_and_follow_only'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to a saved map YAML'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('no_rviz', default_value='false'),
        DeclareLaunchArgument('plan_and_follow_only', default_value='false'),
        DeclareLaunchArgument('managed_startup', default_value='false',
                              description='Session explicitly orders sensors, localization and navigation initialization'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            get_package_share_directory('racecar'), 'rviz', 'racecar_saved_map.rviz')),
        OpaqueFunction(function=setup),
    ])
