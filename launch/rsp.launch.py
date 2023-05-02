import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    wheels_d = LaunchConfiguration('wheels_d')
    wheels_width = LaunchConfiguration('wheels_width')
    c_length = LaunchConfiguration('c_length')
    c_width = LaunchConfiguration('c_width')
    c_height = LaunchConfiguration('c_height')
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    front_camera = LaunchConfiguration('front_camera')
    rear_camera = LaunchConfiguration('rear_camera')
    fl_sensor = LaunchConfiguration('fl_sensor')
    fr_sensor = LaunchConfiguration('fr_sensor')
    sl_sensor = LaunchConfiguration('sl_sensor')
    sr_sensor = LaunchConfiguration('sr_sensor')
    rl_sensor = LaunchConfiguration('rl_sensor')
    rr_sensor = LaunchConfiguration('rr_sensor')
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('super_3d_cart'))

    robot_core_file = os.path.join(pkg_path,'description','robot_core.xacro')
    #xacro.process_file(robot_core_file, mappings={"wheels_d": wheels_d}).toxml()
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file,
                                        ' use_ros2_control:=', use_ros2_control,
                                        ' sim_mode:=', use_sim_time,
                                        ' wheels_d:=', wheels_d,
                                        ' wheels_width:=', wheels_width,
                                        ' c_length:=', c_length,
                                        ' c_width:=', c_width,
                                        ' c_height:=', c_height,
                                        ' front_camera:=', front_camera,
                                        ' rear_camera:=', rear_camera,
                                        ' fl_sensor:=', fl_sensor,
                                        ' fr_sensor:=', fr_sensor,
                                        ' sl_sensor:=', sl_sensor,
                                        ' sr_sensor:=', sr_sensor,
                                        ' rl_sensor:=', rl_sensor,
                                        ' rr_sensor:=', rr_sensor,
                                        ])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),

        node_robot_state_publisher
    ])
