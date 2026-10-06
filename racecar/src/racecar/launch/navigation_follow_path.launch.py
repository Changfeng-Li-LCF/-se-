"""Shared FollowPath settings; with_planner adds continuous single-goal navigation."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
from racecar.planning_geometry import startup_radius, paths


def build_nodes(context):
    with_planner = LaunchConfiguration('with_planner').perform(context).lower() == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time')
    through = LaunchConfiguration('enable_through_poses').perform(context).lower() == 'true'
    rewrites = {'use_sim_time': use_sim_time}
    share = get_package_share_directory('racecar')
    if with_planner:
        rewrites['minimum_turning_radius'] = str(startup_radius(share))
    parameters = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites=rewrites, convert_types=True), allow_substs=True)
    nodes = [Node(package='nav2_controller', executable='controller_server',
                  name='controller_server', output='screen', parameters=[parameters],
                  remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'), ('cmd_vel', 'cmd_vel_nav')])]
    lifecycle_names = ['controller_server']
    if with_planner:
        nodes.append(Node(package='nav2_planner', executable='planner_server',
                          name='planner_server', output='screen', parameters=[parameters],
                          remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))
        lifecycle_names.append('planner_server')
    nodes.append(Node(package='nav2_velocity_smoother', executable='velocity_smoother',
                      name='velocity_smoother', output='screen', parameters=[parameters],
                      remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'car_cmd_vel')]))
    lifecycle_names.append('velocity_smoother')
    if with_planner:
        # Humble creates both navigator interfaces at activation. Give the unused
        # through-poses interface a non-driving tree, so it cannot instantiate the
        # upstream Spin/BackUp recovery clients absent from this lean stack.
        bt_parameters = {
            'default_nav_to_pose_bt_xml': os.path.join(
                share, 'config', 'navigate_live_replanning.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                share, 'config', 'navigate_through_poses_disabled.xml'),
            'plugin_lib_names': [
                'nav2_compute_path_to_pose_action_bt_node',
                'nav2_follow_path_action_bt_node',
                'nav2_rate_controller_bt_node',
                'nav2_pipeline_sequence_bt_node',
                'nav2_goal_updater_node_bt_node',
                'racecar_goal_preference_bt_node',
            ],
            # Action acknowledgement timeout in milliseconds, not a fixed delay
            # and not the planner's computation deadline.
            'goal_updater_topic': '/live_slam/rolling_goal',
            'default_server_timeout': 1000,
            'wait_for_service_timeout': 5000,
        }
        if through:
            bt_parameters['plugin_lib_names'] += [
                'nav2_compute_path_through_poses_action_bt_node',
                'nav2_remove_passed_goals_action_bt_node',
            ]
            bt_parameters['default_nav_through_poses_bt_xml'] = os.path.join(
                share, 'config', 'navigate_marked_route.xml')
        nodes.append(Node(package='nav2_bt_navigator', executable='bt_navigator',
                          name='bt_navigator', output='screen',
                          parameters=[parameters, bt_parameters],
                          remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static'),
                                      ('goal_pose', 'goal_pose_nav')]))
        lifecycle_names.append('bt_navigator')
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen', parameters=[{
                          'use_sim_time': use_sim_time, 'autostart': LaunchConfiguration('autostart').perform(context).lower() == 'true',
                          'node_names': lifecycle_names, 'bond_timeout': 0.0,
                          'attempt_respawn_reconnection': False, 'node_startup_timeout': 15.0}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('enable_through_poses', default_value='false'),
        DeclareLaunchArgument('with_planner', default_value='false',
                              description='Add planner and single-goal BT replanning'),
        OpaqueFunction(function=build_nodes),
    ])
