import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'super_3d_cart'

    wheels_d = LaunchConfiguration('wheels_d')
    wheels_width = LaunchConfiguration('wheels_width')
    c_length = LaunchConfiguration('c_length')
    c_width = LaunchConfiguration('c_width')
    c_height = LaunchConfiguration('c_height')
    world_file = LaunchConfiguration('world_file')
    front_camera = LaunchConfiguration('front_camera')
    rear_camera = LaunchConfiguration('rear_camera')

    fl_sensor = LaunchConfiguration('fl_sensor')
    fr_sensor = LaunchConfiguration('fr_sensor')
    sl_sensor = LaunchConfiguration('sl_sensor')
    sr_sensor = LaunchConfiguration('sr_sensor')
    rl_sensor = LaunchConfiguration('rl_sensor')
    rr_sensor = LaunchConfiguration('rr_sensor')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={
                                        'use_sim_time': 'true',
                                        'use_ros2_control': 'true',
                                        'wheels_d': wheels_d,
                                        'wheels_width': wheels_width,
                                        'c_length': c_length,
                                        'c_width': c_width,
                                        'c_height': c_height,
                                        'front_camera': front_camera,
                                        'rear_camera': rear_camera,
                                        'fl_sensor': fl_sensor,
                                        'fr_sensor': fr_sensor,
                                        'sl_sensor': sl_sensor,
                                        'sr_sensor': sr_sensor,
                                        'rl_sensor': rl_sensor,
                                        'rr_sensor': rr_sensor,
                                       }.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file, 'world': world_file}.items()
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["diff_cont"],
    # )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheels_controller"],
    )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )


    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        controller_spawner,
    ])
