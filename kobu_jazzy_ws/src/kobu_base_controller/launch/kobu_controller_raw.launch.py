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
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.16',
        description='Distance between wheels in meters'
    )
    
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s'
    )
    
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='2.0',
        description='Maximum angular speed in rad/s'
    )
    
    command_rate_arg = DeclareLaunchArgument(
        'command_rate',
        default_value='20',
        description='Rate at which to send commands to the robot (Hz)'
    )
    
    # KOBU base controller node using raw motor control
    kobu_controller_node = Node(
        package='kobu_base_controller',
        executable='kobu_base_controller_raw',
        name='kobu_base_controller',
        output='screen',
        parameters=[{
            'uart_device': LaunchConfiguration('uart_device'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'command_rate': LaunchConfiguration('command_rate'),
        }]
    )
    
    # Return the launch description
    return LaunchDescription([
        uart_device_arg,
        baud_rate_arg,
        cmd_vel_timeout_arg,
        wheel_separation_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        command_rate_arg,
        kobu_controller_node,
    ])