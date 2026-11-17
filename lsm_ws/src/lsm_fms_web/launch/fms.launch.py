import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Web Server Node
    web_server_cmd = Node(
        package='lsm_fms_web',
        executable='web_server',
        name='web_server',
        output='screen'
    )

    # 2. Telemetry Bridge Node
    telemetry_bridge_cmd = Node(
        package='lsm_fms_web',
        executable='telemetry_bridge',
        name='telemetry_bridge',
        output='screen'
    )

    # 3. Route Server (Sequential Dispatcher) Node
    route_server_cmd = Node(
        package='lsm_fms_web',
        executable='route_server',
        name='route_server',
        output='screen'
    )

    ld = LaunchDescription()
    
    ld.add_action(web_server_cmd)
    ld.add_action(telemetry_bridge_cmd)
    ld.add_action(route_server_cmd)

    return ld
