#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Create a ROS logger
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur5_demo");

int main(int argc, char * argv[]){
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pnp_node");
    RCLCPP_INFO(LOGGER, "Initialize node");

    // We spin up a SingleThreadedExecutor so we can get current joint values later
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();
    RCLCPP_INFO(LOGGER, "Created executor");

    // Create the MoveIt Move Group Interface for UR5
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group = MoveGroupInterface(node, "ur_manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// Adding table
    auto const add_object1 = [frame_id = move_group.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject add_object1;
        add_object1.header.frame_id = frame_id;
        add_object1.id = "table";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 1.5;
        primitive.dimensions[primitive.BOX_Y] = 1.5;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = 0.5;
        table_pose.position.y = 0.0;
        table_pose.position.z = -0.1;

        add_object1.primitives.push_back(primitive);
        add_object1.primitive_poses.push_back(table_pose);
        add_object1.operation = add_object1.ADD;

        return add_object1;}();
    planning_scene_interface.applyCollisionObject(add_object1);

// Go to initial position
    move_group.setJointValueTarget(move_group.getNamedTargetValues("home"));

    // Create a plan to that target pose and check if that plan is successful
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success){move_group.execute(my_plan);}
    RCLCPP_INFO(LOGGER, "Executing plan 0 (home position) %s", success ? "" : "FAILED");

// Adding box to force it to create a better plan
    auto const add_object2 = [frame_id = move_group.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject add_object2;
        add_object2.header.frame_id = frame_id;
        add_object2.id = "box";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 1.5;
        primitive.dimensions[primitive.BOX_Z] = 0.1;

        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.5;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.2;

        add_object2.primitives.push_back(primitive);
        add_object2.primitive_poses.push_back(box_pose);
        add_object2.operation = add_object2.ADD;

        return add_object2;}();
    planning_scene_interface.applyCollisionObject(add_object2);

// Move to the object using pose goal
    geometry_msgs::msg::Pose target_pose1;

    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(target_pose1.orientation , q_orig);
    double r=3.14159, p=0, y=0;  // Rotate the previous pose by 180* about X
    q_rot.setRPY(r, p, y);
    q_new = q_rot*q_orig;  // Calculate the new orientation
    q_new.normalize();
    tf2::convert(q_new, target_pose1.orientation);

    target_pose1.position.x = 0.35;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;

// Set the target pose
    move_group.setPoseTarget(target_pose1, "tool0");
    move_group.setPlanningTime(10.0); // Making sure that there will be enough time for planning
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success){move_group.execute(my_plan);}
    RCLCPP_INFO(LOGGER, "Executing plan 1 (pose goal) %s", success ? "" : "FAILED");

// Remove the box
    auto const remove_object2 = [frame_id = move_group.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject remove_object2;
        remove_object2.header.frame_id = frame_id;
        remove_object2.id = "box";
        remove_object2.operation = remove_object2.REMOVE;
        return remove_object2;}();
    planning_scene_interface.applyCollisionObject(remove_object2); // removing the box

// Pick the object using cartesian path
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
    my_plan.trajectory_ = trajectory;
    move_group.execute(my_plan);
    RCLCPP_INFO(LOGGER, "Executing plan 2 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

// Move the object using joint goal
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();
    joint_group_positions[0] += 2.0;
    move_group.setJointValueTarget(joint_group_positions);
    success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success){move_group.execute(my_plan);}
    RCLCPP_INFO(LOGGER, "Executing plan 3 (joint space goal) %s", success ? "" : "FAILED");

// Place the object using cartesian path
    geometry_msgs::msg::PoseStamped current_pose;
    current_pose = move_group.getCurrentPose();
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
    my_plan.trajectory_ = trajectory;
    move_group.execute(my_plan);
    RCLCPP_INFO(LOGGER, "Executing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

    rclcpp::shutdown();
    return 0;
}   