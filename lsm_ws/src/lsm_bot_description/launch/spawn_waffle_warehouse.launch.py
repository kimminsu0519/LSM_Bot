import os
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_robot_description(urdf_path: str, slip_mode: bool) -> str:
    with open(urdf_path, 'r', encoding='utf-8') as infp:
        robot_desc = infp.read()

    if slip_mode:
        # Override wheel_left_link (mu1/mu2=0.35) and wheel_right_link (mu1/mu2=0.28) to induce asymmetric wheel slip
        robot_desc = re.sub(
            r'(<gazebo reference="wheel_left_link">\s*<mu1>)[0-9.]+(</mu1>\s*<mu2>)[0-9.]+(</mu2>)',
            r'\g<1>0.35\g<2>0.35\g<3>',
            robot_desc,
            count=1,
        )
        robot_desc = re.sub(
            r'(<gazebo reference="wheel_right_link">\s*<mu1>)[0-9.]+(</mu1>\s*<mu2>)[0-9.]+(</mu2>)',
            r'\g<1>0.28\g<2>0.28\g<3>',
            robot_desc,
            count=1,
        )
        print("[spawn_waffle_warehouse] slip_mode=true -> Asymmetric low-friction slip enabled (wheel_left=0.35, wheel_right=0.28)")
    else:
        print("[spawn_waffle_warehouse] slip_mode=false -> Symmetric normal warehouse tire friction enabled (wheel_left=1.0, wheel_right=1.0)")

    return robot_desc


def _launch_setup(context, *args, **kwargs):
    pkg_lsm_bot_description = get_package_share_directory('lsm_bot_description')
    urdf_path = os.path.join(pkg_lsm_bot_description, 'urdf', 'turtlebot3_waffle_gz.urdf')

    slip_mode_str = LaunchConfiguration('slip_mode').perform(context).strip().lower()
    slip_mode = slip_mode_str in ('true', '1', 'yes')
    robot_desc = _build_robot_description(urdf_path, slip_mode)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )
    return [robot_state_publisher_node]


def generate_launch_description():
    pkg_lsm_bot_description = get_package_share_directory('lsm_bot_description')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Parent directory containing turtlebot3_description for GZ_SIM_RESOURCE_PATH
    tb3_parent_dir = os.path.abspath(os.path.join(pkg_turtlebot3_description, '..'))
    current_gz_res = os.getenv('GZ_SIM_RESOURCE_PATH', '')
    new_gz_res = f"{tb3_parent_dir}:{current_gz_res}" if current_gz_res else tb3_parent_dir

    set_gz_resource_env = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_res)
    set_ign_resource_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', new_gz_res)
    set_tb3_model_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # World Path
    world_path = os.path.join(pkg_lsm_bot_description, 'worlds', 'lsm_custom_warehouse.world')
    world = LaunchConfiguration('world')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to world model file to load'
    )

    declare_slip_mode_cmd = DeclareLaunchArgument(
        'slip_mode',
        default_value='false',
        description='Enable asymmetric wheel-slip friction (wheel_left=0.35, wheel_right=0.28) for EKF benchmark'
    )

    gui_config_path = os.path.join(pkg_lsm_bot_description, 'config', 'lsm_gui.config')

    # Start Gazebo Sim
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r --gui-config ', gui_config_path, ' ', world]}.items()
    )

    # Spawn Waffle on Charging Pad (2.4, 0.3, 0.1, yaw=3.14159 / 180 deg facing LEFT)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'LSM01_Waffle',
            '-topic', 'robot_description',
            '-x', '2.4',
            '-y', '0.3',
            '-z', '0.1',
            '-Y', '3.14159'
        ],
        output='screen'
    )

    bridge_config_path = os.path.join(pkg_lsm_bot_description, 'config', 'ros_gz_bridge.yaml')

    # ROS-GZ Bridge for Clock & Cmd_vel & Scan & Odom & IMU & GroundTruth
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config_path,
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(set_gz_resource_env)
    ld.add_action(set_ign_resource_env)
    ld.add_action(set_tb3_model_env)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_slip_mode_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(OpaqueFunction(function=_launch_setup))
    ld.add_action(spawn_robot_node)
    ld.add_action(bridge_node)

    return ld
