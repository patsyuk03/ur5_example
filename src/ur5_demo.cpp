#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur5_demo");

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    RCLCPP_INFO(LOGGER, "Initialize node");

    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pnp_node", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    RCLCPP_INFO(LOGGER, "Created executor");

    static const std::string PLANNING_GROUP = "ur_manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

// Move to the object
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1, "tool0");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(my_plan);
    }
    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");\

// Pick the object
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose1);
    target_pose1.position.z -= 0.2;
    waypoints.push_back(target_pose1);  // down
    target_pose1.position.z += 0.2;
    waypoints.push_back(target_pose1);  // up
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

// Move the object
    moveit::core::RobotState start_state(*move_group.getCurrentState());
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] += 2.0;
    move_group.setJointValueTarget(joint_group_positions);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(my_plan);
    }
    RCLCPP_INFO(LOGGER, "Visualizing plan 3 (joint space goal) %s", success ? "" : "FAILED");

// Place the object
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose();
    RCLCPP_INFO(LOGGER, "Current pose x: %f", current_pose.pose.position.x);
    RCLCPP_INFO(LOGGER, "Current pose y: %f", current_pose.pose.position.y);
    RCLCPP_INFO(LOGGER, "Current pose z: %f", current_pose.pose.position.z);
    waypoints.clear();
    target_pose1.position.x = current_pose.pose.position.x;
    target_pose1.position.y = current_pose.pose.position.y;
    target_pose1.position.z = current_pose.pose.position.z;
    waypoints.push_back(target_pose1);
    target_pose1.position.z -= 0.2;
    waypoints.push_back(target_pose1);  // down
    target_pose1.position.z += 0.2;
    waypoints.push_back(target_pose1);  // up
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    rclcpp::shutdown();
    return 0;
}   