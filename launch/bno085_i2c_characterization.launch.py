from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('bno08x_driver'), 
        'config',
        'bno085_i2c_characterization.yaml'
    )

    return LaunchDescription([
        # Pick a different profile with config:=<path>, e.g. the lean Allan variance profile:
        # config:=$(ros2 pkg prefix --share bno08x_driver)/config/bno085_i2c_allan.yaml
        DeclareLaunchArgument('config', default_value=default_config,
                              description='Driver parameter file'),
        Node(
            package='bno08x_driver',  
            executable='bno08x_driver',  
            name='bno08x_driver',
            output='screen',
            parameters=[LaunchConfiguration('config')]
        ),
    ])
