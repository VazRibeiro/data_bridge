from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='WebSocket port for Protocol Buffer bridge'
    )
    
    return LaunchDescription([
        port_arg,
        
        # Protocol Buffer Bridge (C++)
        Node(
            package='data_bridge',
            executable='proto_bridge',
            name='proto_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port')
            }]
        ),
    ])
