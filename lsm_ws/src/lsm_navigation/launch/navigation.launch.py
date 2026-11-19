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
    default_ekf_config_path = os.path.join(pkg_lsm_navigation, 'config', 'ekf.yaml')
    custom_rviz_path = os.path.join(pkg_lsm_navigation, 'rviz', 'lsm_nav2_view.rviz')
    default_rviz_path = custom_rviz_path if os.path.exists(custom_rviz_path) else os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    # Launch Configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slip_mode = LaunchConfiguration('slip_mode')
    rviz_config = LaunchConfiguration('rviz_config')

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

    declare_ekf_params_file_cmd = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=default_ekf_config_path,
        description='Full path to the EKF parameters file (default: 6-axis gyro ekf.yaml; ablation: ekf_raw_odom_ref.yaml)'
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

    declare_slip_mode_cmd = DeclareLaunchArgument(
        'slip_mode',
        default_value='false',
        description='Enable asymmetric wheel-slip friction (wheel_left=0.35, wheel_right=0.28) for EKF benchmark'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_path,
        description='Full path to RViz2 configuration file'
    )

    # 1. Include Gazebo World & Robot Spawn Launch
    spawn_warehouse_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lsm_bot_description, 'launch', 'spawn_waffle_warehouse.launch.py')
        ),
        launch_arguments={'slip_mode': slip_mode}.items()
    )

    # 1.5 EKF Sensor Fusion Node (robot_localization: fuses /odom + /imu -> /odometry/filtered & odom->base_footprint TF)
    ekf_node_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 1.6 5-Stage Logistics AMR Finite State Machine Node (C++17 rclcpp)
    fsm_node_cmd = Node(
        package='lsm_navigation',
        executable='amr_state_machine_node',
        name='amr_state_machine_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 1.7 50Hz Closed-Loop PID Precision Docking & 360° LiDAR Safety Supervisor Node (C++17 rclcpp)
    docking_node_cmd = Node(
        package='lsm_navigation',
        executable='precision_docking_node',
        name='precision_docking_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 1.8 206-Node Topological A* Route Planner Server Node (C++17 rclcpp)
    route_planner_cpp_cmd = Node(
        package='lsm_navigation',
        executable='topological_route_planner_node',
        name='topological_route_planner_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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

    # 3. RViz2 Node with Warehouse Full-View Config
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_ekf_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slip_mode_cmd)
    ld.add_action(declare_rviz_config_cmd)

    ld.add_action(spawn_warehouse_cmd)
    ld.add_action(ekf_node_cmd)
    ld.add_action(fsm_node_cmd)
    ld.add_action(docking_node_cmd)
    ld.add_action(route_planner_cpp_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld
