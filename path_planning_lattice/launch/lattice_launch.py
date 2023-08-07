import launch
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
import yaml
import ament_index_python.packages
import sys

def generate_launch_description():
    # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    # print(sys.argv[0])
    # print(__file__)

    # print(get_package_share_directory('path_planning_lattice'))
    
    lattice_parameters_configuration = os.path.join(get_package_share_directory('path_planning_lattice'), './../../../../src/path_planning_lattice/config', 'lattice_parameters_configuration.yaml')
    # print(lattice_parameters_configuration)

    rviz_config_dir = os.path.join(get_package_share_directory('path_planning_lattice'), 'rviz', 'lattice_vis.rviz')

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable("USER"), '_'],
            description='Prefix for node names'
        ),
        Node(
            package='path_planning_lattice',
            executable='lattice_planner',
            name='lattice_planner',
            parameters=[lattice_parameters_configuration],
            # remappings=None,
            # arguments=None,
            output='screen',
        ),
        Node(package='rviz2',
             executable='rviz2',
             output='screen',
             arguments=['-d', rviz_config_dir]),
    ])
