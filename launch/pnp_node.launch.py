import os, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("ur_type", default_value="ur5"))
    ld.add_action(DeclareLaunchArgument("robot_ip", default_value="yyy.yyy.yyy.yyy"))
    ld.add_action(DeclareLaunchArgument("use_fake_hardware", default_value="true"))

    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # ur5_example_yaml_file_name = (
    #     get_package_share_directory("ur5_example") + "/config/ur5_pnp.yaml"
    # )

    joint_limit_params = PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"])
    kinematics_params = PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"])
    physical_params = PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"])
    visual_params = PathJoinSubstitution([FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"])
    script_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"])
    input_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"])
    output_recipe_filename = PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"])

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")])," ",
            PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"])," ",
            "robot_ip:=",robot_ip," ",
            "joint_limit_params:=",joint_limit_params," ",
            "kinematics_params:=",kinematics_params," ",
            "physical_params:=",physical_params," ",
            "visual_params:=",visual_params," ",
            "name:=","ur"," ",
            "script_filename:=",script_filename," ",
            "input_recipe_filename:=",input_recipe_filename," ",
            "output_recipe_filename:=",output_recipe_filename," ",
            "use_fake_hardware:=",use_fake_hardware," ",
        ]
    )
    robot_description_semantic = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")])," ",
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"])," ",
            "name:=","ur"," ",
        ]
    )
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    ld.add_action(Node(
        package="ur5_example",
        executable="ur5_demo",
        output="screen",
        parameters=[{"robot_description": robot_description},
                    {"robot_description_semantic": robot_description_semantic},
                    {"robot_description_kinematics": kinematics_yaml},
                    {"use_sim_time": True},
                    # ompl_planning_pipeline_config,
                    # moveit_controllers,
                    # ur5_example_yaml_file_name
                    ],   
    ))

    return ld