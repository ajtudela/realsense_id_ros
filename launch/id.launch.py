#!/usr/bin/env python3

'''
    Launches a RealSense ID node.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Getting directories and launch-files
    realsense_id_dir = get_package_share_directory('realsense_id_ros')
    default_params_file = os.path.join(realsense_id_dir, 'params', 'default_params.yaml')
    default_database_param_file = os.path.join(realsense_id_dir, 'database_filepath', 'faceprints.json')

    # Create the launch configuration variables.
    params_file = LaunchConfiguration('params', default=default_params_file)

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
    #    'database_filepath': default_database_param_file, 
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Map these variables to arguments: can be set from the command line or a default will be used
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Full path to the ROS2 parameters file with id configuration'
    )

    # Prepare the laser segmentation node.
    realsense_id_node = Node(
        package = 'realsense_id_ros',
        namespace = '',
        executable = 'realsense_id_ros_node',
        name = 'realsense_id',
        parameters=[configured_params],
        emulate_tty = True
    )

    return LaunchDescription([
        declare_params_file_arg,
        realsense_id_node
    ])