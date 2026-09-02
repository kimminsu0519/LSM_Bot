import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Load URDF for Waffle
    urdf_file_name = 'turtlebot3_waffle.urdf'
    urdf_path = os.path.join(pkg_turtlebot3_description, 'urdf', urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Robot State Publisher Node
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

    # Start Gazebo Sim
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # Spawn Waffle on Charging Pad (-68.0, -16.0, 0.2)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'LSM01_Waffle',
            '-topic', 'robot_description',
            '-x', '-21.0',
            '-y', '-5.1',
            '-z', '0.2'
        ],
        output='screen'
    )

    # ROS-GZ Bridge for Clock & Cmd_vel & Scan
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(set_gz_resource_env)
    ld.add_action(set_ign_resource_env)
    ld.add_action(set_tb3_model_env)
    ld.add_action(declare_world_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_robot_node)
    ld.add_action(bridge_node)

    return ld
