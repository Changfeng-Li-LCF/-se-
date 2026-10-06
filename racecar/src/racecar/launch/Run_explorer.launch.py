"""
Run_explorer.launch.py - racecar Cartographer 自主探索模式
移植自仿真项目 turtlebot3_ws 的 explorer 方案

架构（与仿真完全一致）：
  Laser + IMU → Cartographer → map→odom→base_footprint TF + /map
  Cartographer /map → Nav2 (navigation_launch.py, 无 AMCL)

TF 链路：
  map → odom → base_footprint → laser_link / IMU_link
  (map→odom 和 odom→base_footprint 全部由 Cartographer 发布)

启动方式：
  ros2 launch racecar Run_explorer.launch.py
"""

import os
from racecar.planning_geometry import boundary_guard_enabled
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch_ros.actions


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    racecar_dir = get_package_share_directory('racecar')
    launch_dir = os.path.join(racecar_dir, 'launch')

    lslidar_driver_share_dir = get_package_share_directory('lslidar_driver')
    imu_launch_share_dir = get_package_share_directory('imu_get')

    # Nav2 参数文件（Cartographer 专用，无 AMCL）
    nav_carto_param = os.path.join(racecar_dir, 'config', 'nav_carto.yaml')

    # RViz2 配置
    rviz_config_dir = LaunchConfiguration('rviz_config')

    lean_navigation = PythonExpression(["'", LaunchConfiguration('follow_path_only'),
        "' == 'true' or '", LaunchConfiguration('plan_and_follow_only'), "' == 'true'"])

    return LaunchDescription([
        Node(package='racecar', executable='scan_motion_odometry.py',
             name='scan_motion_odometry', output='log',
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
             parameters=[{'driver_config': os.path.join(get_package_share_directory('racecar'),
                                                        'config', 'driver_calibration.yaml')}]),
        DeclareLaunchArgument('plan_and_follow_only', default_value='false',
                              description='Only planner, controller and velocity smoother for one recorded trial'),
        DeclareLaunchArgument('enable_navigation', default_value='true'),
        DeclareLaunchArgument('navigation_autostart', default_value='true',
                              description='Session-managed runs activate navigation after sensors and SLAM are ready'),
        DeclareLaunchArgument('cartographer_config', default_value='racecar_2d.lua'),
        DeclareLaunchArgument('publish_map', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(
            racecar_dir, 'rviz', 'racecar_cartographer.rviz')),

        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('follow_path_only', default_value='false',
                              description='Load only controller and velocity smoother for recorded path tests'),

        Node(package='racecar', executable='stop_report_recorder.py',
             name='stop_report_recorder', output='log'),
        Node(package='racecar', executable='map_boundary_guard.py',
             name='map_boundary_guard', output='screen',
             condition=IfCondition(str(boundary_guard_enabled(get_package_share_directory('racecar'))).lower()),
             parameters=[os.path.join(get_package_share_directory('racecar'),
                                      'config', 'map_boundary_guard.yaml'), {'require_map': True, 'require_map_updates': LaunchConfiguration('publish_map')}]),

        # ==========================================
        # 第一层：底盘驱动 + 传感器
        # ==========================================

        # 底盘驱动
        Node(
            package='racecar_driver',
            executable='racecar_driver_node',
            name='racecar_driver',
            parameters=[os.path.join(get_package_share_directory('racecar'),
                                     'config', 'driver_calibration.yaml')],
        ),

        # 关节状态发布
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),

        # 雷达 LSN10
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lslidar_driver_share_dir, 'launch', 'lsn10_launch.py')
            )
        ),

        # IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(imu_launch_share_dir, 'launch', 'imu_get.launch.py')
            )
        ),

        # 静态 TF：base_footprint → laser_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2laser_link',
            arguments=['0.07', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'laser_link']
        ),

        # 静态 TF：base_footprint → IMU_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2imu_link',
            arguments=['0.1653', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'IMU_link']
        ),

        # ==========================================
        # 第二层：Cartographer SLAM
        # 和仿真方案完全一致：Laser + IMU + 独立连续扫描运动预测
        # Cartographer 自行提供 odom frame，发布：
        #   map → odom → base_footprint 的完整 TF
        # 延迟 3 秒启动，等传感器就绪
        # ==========================================
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'cartographer.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'configuration_basename': LaunchConfiguration('cartographer_config'),
                        'publish_map': LaunchConfiguration('publish_map'),
                    }.items(),
                ),
                # Preserve SLAM pose/trajectory output; motion control uses scan_motion_feedback.
                Node(
                    package='racecar',
                    executable='cartographer_odom_bridge.py',
                    name='cartographer_odom_bridge',
                    output='log',
                    parameters=[{'use_sim_time': use_sim_time,
                                 'odom_frame': 'odom', 'base_frame': 'base_footprint',
                                 'odom_topic': '/odom', 'publish_rate': 20.0}],
                ),
            ]
        ),

        # ==========================================
        # 第三层：Nav2 导航栈
        # 使用裁剪版 navigation_no_smoother.launch.py
        # 去掉 smoother_server，避免实车上生命周期切换异常
        # 延迟 4 秒启动，等 Cartographer 就绪
        # ==========================================
        GroupAction(condition=IfCondition(LaunchConfiguration('enable_navigation')), actions=[TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'navigation_no_smoother.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': nav_carto_param,
                        'use_composition': 'False',
                        'autostart': LaunchConfiguration('navigation_autostart'),
                    }.items(),
                    condition=UnlessCondition(lean_navigation),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'navigation_follow_path.launch.py')
                    ),
                    launch_arguments={'use_sim_time': use_sim_time,
                                      'params_file': nav_carto_param,
                                      'with_planner': LaunchConfiguration('plan_and_follow_only'),
                                      'autostart': LaunchConfiguration('navigation_autostart')}.items(),
                    condition=IfCondition(lean_navigation),
                ),
            ]
        )]),

        # ==========================================
        # RViz2 可视化（默认启用）
        # 使用 --no-rviz 禁用：bash explorer.sh --no-rviz
        # ==========================================
        Node(
            package='rviz2',
            executable='/home/bianbu/.local/lib/racecar-rviz-relay/rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='log',
            condition=UnlessCondition(LaunchConfiguration('no_rviz', default='false')),
        ),
    ])
