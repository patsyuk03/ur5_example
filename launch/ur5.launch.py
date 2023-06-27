import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("ur_type", default_value="ur5"))
    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="xxx.xxx.x.xxx"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="true"))

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch/ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'initial_joint_controller':'joint_trajectory_controller',
            'launch_rviz': 'false',
        }.items(),
        condition=IfCondition(use_fake_hardware)
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'), 'launch/ur_control.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',
        }.items(),
        condition=UnlessCondition(use_fake_hardware)
    ))
    # ld.add_action(fake_hardware)
    # ld.add_action(real_robot)


    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_moveit_config'), 'launch/ur_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': ur_type,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'true',
        }.items()
    ))
    return ld
