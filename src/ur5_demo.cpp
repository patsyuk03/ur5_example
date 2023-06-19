#include <rclcpp/rclcpp.hpp>
// #include <moveit/moveit_cpp/moveit_cpp.h>
// #include <moveit/moveit_cpp/planning_component.h>
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
    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

    // using moveit::planning_interface::MoveGroupInterface;
    // auto move_group = MoveGroupInterface(node, "ur_manipulator");

    // planning_components->setStartStateToCurrentState();
    // RCLCPP_INFO(LOGGER, "setStartStateToCurrentState");

    geometry_msgs::msg::PoseStamped target_pose1;
    target_pose1.header.frame_id = "base_link";
    target_pose1.pose.orientation.w = 1.0;
    target_pose1.pose.position.x = 0.28;
    target_pose1.pose.position.y = -0.2;
    target_pose1.pose.position.z = 0.5;
    move_group.setPoseTarget(target_pose1, "tool0");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(my_plan);
    }

    moveit::core::RobotState start_state(*move_group.getCurrentState());
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] += 1.0;  // radians
    move_group.setJointValueTarget(joint_group_positions);

    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        move_group.execute(my_plan);
    }

    // const moveit::core::JointModelGroup* joint_model_group =
    //     move_group.getCurrentState()->getJointModelGroup("ur_manipulator");
    // moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    // geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    // geometry_msgs::msg::Pose target_pose = current_pose.pose;
    // target_pose.position.x = 0.3;
    // target_pose.position.z = 0.5;
    // move_group.setPoseTarget(target_pose);
    // bool success = static_cast<bool>(move_group.plan(plan));
    // if(success) {
    //     move_group.execute(plan);
    // } else {
    //     RCLCPP_ERROR(logger, "Planing failed!");
    // }

    // std::vector<double> joint_values = move_group.getCurrentJointValues();
    // std::vector<double> joint_values;
    // current_state->copyJointGroupPositions(joint_model_group, joint_values);
    // RCLCPP_INFO(logger, "Joint state:%f", joint_values[0]);
    // joint_values[0] += 1;
    // move_group.setJointValueTarget(joint_values);
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // bool success = static_cast<bool>(move_group.plan(plan));
    // if(success) {
    //     move_group.execute(plan);
    // } else {
    //     RCLCPP_ERROR(logger, "Planing failed!");
    // }


    rclcpp::shutdown();
    return 0;
}   