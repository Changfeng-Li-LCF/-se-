import os
from racecar.planning_geometry import boundary_guard_enabled
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('racecar')
    launch_dir = os.path.join(bringup_dir, 'launch')
        
    ekf_config = Path(get_package_share_directory('racecar'), 'config', 'ekf.yaml')

    ekf_carto_config = Path(get_package_share_directory('racecar'), 'config', 'ekf_carto.yaml')
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')

    # imu
    lslidar_driver_share_dir = get_package_share_directory('lslidar_driver')
    imu_launch_share_dir = get_package_share_directory('imu_get')
    laser_odom_launch_share_dir = get_package_share_directory('rf2o_laser_odometry')
 #   imu_launch_share_dir = get_package_share_directory('hipnuc_imu')
    # 声明参数
#    imu_package_arg = DeclareLaunchArgument(
#        'imu_package', default_value='spec',
#        description='package type [spec, 0x91]'
#    )

#   imu_package = LaunchConfiguration('imu_package')
            
     
    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py')),
        launch_arguments={'carto_slam':carto_slam}.items(),            
    )
                                                            
        
    # imu_filter_node =  launch_ros.actions.Node(
    #     package='hipnuc_imu',
    #     executable='imu_filter_node',
    #     name='imu_filter_node',
    # )

        
                           
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher', 
        executable='joint_state_publisher', 
        name='joint_state_publisher',
    )
    # rviz_dir = os.path.join(get_package_share_directory('racecar'), 'rviz', 'lslidar.rviz')
    rviz_dir = os.path.join(get_package_share_directory('racecar'), 'rviz', 'nav2_default_view.rviz')
    

    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='/home/bianbu/.local/lib/racecar-rviz-relay/rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen')

    return LaunchDescription([
        Node(package='racecar', executable='scan_motion_odometry.py',
             name='scan_motion_odometry', output='log',
             additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
             parameters=[{'driver_config': os.path.join(get_package_share_directory('racecar'),
                                                        'config', 'driver_calibration.yaml')}]),
        Node(package='racecar', executable='stop_report_recorder.py',
             name='stop_report_recorder', output='log'),
        Node(package='racecar', executable='map_boundary_guard.py',
             name='map_boundary_guard', output='screen',
             condition=IfCondition(str(boundary_guard_enabled(get_package_share_directory('racecar'))).lower()),
             parameters=[os.path.join(get_package_share_directory('racecar'),
                                      'config', 'map_boundary_guard.yaml'), {'require_map': False, 'require_map_updates': False}]),
        robot_ekf,
        carto_slam_dec,joint_state_publisher_node,
# rviz_node,
# imu_package_arg,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lslidar_driver_share_dir, 'launch', 'lsn10_launch.py')
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(imu_launch_share_dir, 'launch', 'imu_get.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(laser_odom_launch_share_dir, 'launch', 'rf2o_laser_odometry.launch.py')
            )
        ),
        # Include IMU launch
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(
#                [os.path.join(imu_launch_share_dir, 'launch', 'imu_'), imu_package, '_msg.launch.py']
#            )
#        ),

        # imu_filter_node,

#        Node(
#            package='encoder',
#            executable='encoder_node',
#            name='encoder_vel',
#            output='screen',

#       ),
        Node(
            package='racecar_driver',
            executable='racecar_driver_node',
            name='racecar_driver',
            parameters=[os.path.join(get_package_share_directory('racecar'),
                                     'config', 'driver_calibration.yaml')],
        ),

        # 仅保留传感器到 base_footprint 的 TF
        # 注意：不要在此处发布 map->odom 或 odom->base_footprint，因为远程主机的 cartographer 启动脚本已经在发布它们了
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2laser_link',
            arguments=['0.07', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'laser_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2imu_link',
            arguments=['0.1653', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'IMU_link']
        )
    ])
