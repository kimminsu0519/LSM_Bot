import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_lsm_navigation = get_package_share_directory('lsm_navigation')
    pkg_lsm_bot_description = get_package_share_directory('lsm_bot_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Default Paths
    default_map_path = os.path.join(pkg_lsm_navigation, 'maps', 'lsm_warehouse_map.yaml')
    default_params_path = os.path.join(pkg_lsm_navigation, 'config', 'nav2_params.yaml')

    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Declare Arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    # 1. Include Gazebo World & Robot Spawn Launch
    spawn_warehouse_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lsm_bot_description, 'launch', 'spawn_waffle_warehouse.launch.py')
        )
    )

    # 2. Include Nav2 Bringup Launch
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )

    # 3. RViz2 Node with Nav2 Config
    rviz_config_dir = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)

    ld.add_action(spawn_warehouse_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
