import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('kobu_zed_driver')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'zed2i.yaml')
    
    # Launch arguments
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model (zed2i for this project)'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed',
        description='Camera name for topic namespace'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the configuration file'
    )
    
    # ZED camera node
    zed_camera_node = Node(
        package='kobu_zed_driver',
        executable='zed_camera_node',
        name='zed_camera_driver',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Standard ROS2 topic remappings for compatibility
            ('zed/left/image_rect_color', 'camera/color/image_raw'),
            ('zed/depth/depth_registered', 'camera/depth/image_rect_raw'),
            ('zed/point_cloud/cloud_registered', 'camera/depth/color/points'),
            ('zed/imu/data', 'camera/imu'),
        ]
    )
    
    return LaunchDescription([
        camera_model_arg,
        camera_name_arg,
        config_file_arg,
        zed_camera_node
    ])