import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("ur_type", default_value="ur5"))
    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="true"))

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur5_example'), 'launch/ur5.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
        }.items()
    ))
    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ur5_example'), 'launch/pnp_node.launch.py')
    #     ),
    #     launch_arguments={
    #         'ur_type': ur_type,
    #         'robot_ip': robot_ip,
    #         'use_fake_hardware': use_fake_hardware,
    #     }.items()
    # ))
    return ld
