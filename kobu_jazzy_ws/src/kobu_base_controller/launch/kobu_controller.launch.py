from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    uart_device_arg = DeclareLaunchArgument(
        'uart_device',
        default_value='/dev/ttyUSB0',
        description='UART device to use for communication with the base'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for UART communication'
    )
    
    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='0.5',
        description='Timeout in seconds to stop the robot if no cmd_vel message is received'
    )
    
    # KOBU base controller node
    kobu_controller_node = Node(
        package='kobu_base_controller',
        executable='kobu_base_controller',
        name='kobu_base_controller',
        output='screen',
        parameters=[{
            'uart_device': LaunchConfiguration('uart_device'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout')
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        uart_device_arg,
        baud_rate_arg,
        cmd_vel_timeout_arg,
        kobu_controller_node,
    ])