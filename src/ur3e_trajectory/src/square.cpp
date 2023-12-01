#include "../include/square.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#define ROBOT_MIN_REACH_XY 0.235
#define ROBOT_MAX_REACH_XY 0.57
#define ROBOT_MIN_REACH_XZ 0.175
#define ROBOT_MAX_REACH_XZ 0.6

geometry_msgs::Transform GetRobotBasePose()
{
    ros::NodeHandle robot_base_tf_subscriber;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;

    try{
      transformStamped = tfBuffer.lookupTransform("world", "base_link",
                               ros::Time::now(), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }

    return transformStamped.transform;
}

void MoveBy(double x, double y, double z,
                 moveit::planning_interface::MoveGroupInterface &arm_move_group)
{
    std::string reference_frame {arm_move_group.getPlanningFrame()};
    geometry_msgs::Pose start_pose {arm_move_group.getCurrentPose().pose};

    std::vector<geometry_msgs::Pose> waypoints;

    auto destination {start_pose};
    destination.position.x += x;
    destination.position.y += y;
    destination.position.z += z;

    waypoints.push_back(destination);

    auto trajectory {ArmController::planCartesianPath(start_pose, waypoints,
                                                      reference_frame,
                                                      arm_move_group)};

    arm_move_group.execute(trajectory);
}

void DrawASquareXY(double side_length, const geometry_msgs::Vector3& center_point,
                 MoveitPlanning::PlanningOptions &planning_options,
                 moveit::planning_interface::MoveGroupInterface &arm_move_group)
{
    ros::NodeHandle n;
    // Set the arm in a position to draw a shape
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = M_PI_2;
    joint_targets["shoulder_lift_joint"] = -M_PI_2;
    joint_targets["shoulder_pan_joint"] = M_PI_2;
    joint_targets["wrist_1_joint"] = -M_PI_2;
    joint_targets["wrist_2_joint"] = -M_PI_2;
    joint_targets["wrist_3_joint"] = 0;

    bool joint_plan_success {
        ArmController::planToJointTargets(planning_options, arm_move_group,
                                          joint_plan, joint_targets)};

    if (joint_plan_success) {
        ROS_INFO("Getting ready to draw a square...");
        arm_move_group.execute(joint_plan);
    }

    // Adjust end-effector to the square center to begin drawing at
    auto current_point {arm_move_group.getCurrentPose().pose.position};
    auto adjust_x {center_point.x - current_point.x};
    auto adjust_y {center_point.y - current_point.y};
    auto adjust_z {center_point.z - current_point.z};
    MoveBy(adjust_x, adjust_y, adjust_z, arm_move_group);

    auto reference_frame {arm_move_group.getPlanningFrame()};
    auto start_pose {arm_move_group.getCurrentPose().pose};

    std::vector<geometry_msgs::Pose> waypoints;
    double radius {(side_length * sqrt(2)) / 2};

    // Generate waypoints for a square trajectory
    for (double theta {0}; theta <= 2*M_PI; theta += M_PI/180) {
        geometry_msgs::Pose point_on_square {start_pose};

        double x_component {std::clamp(radius * cos(theta), -side_length/2, side_length/2)};
        double y_component {std::clamp(radius * sin(theta), -side_length/2, side_length/2)};

        point_on_square.position.x = start_pose.position.x + x_component;
        point_on_square.position.y = start_pose.position.y + y_component;

        waypoints.push_back(point_on_square);
    }

    // Return back to the square center
    waypoints.push_back(start_pose);

    auto square_trajectory {ArmController::planCartesianPath(start_pose,
                                                             waypoints,
                                                             reference_frame,
                                                             arm_move_group)};

    ROS_INFO("Trying to draw a square on XY plane...");
    // Draw the square
    n.setParam("/record_pose", true);
    arm_move_group.execute(square_trajectory);
    n.setParam("/record_pose", false);
}

void DrawASquareXZ(double side_length, const geometry_msgs::Vector3& center_point,
                 MoveitPlanning::PlanningOptions &planning_options,
                 moveit::planning_interface::MoveGroupInterface &arm_move_group)
{
    ros::NodeHandle n;
    // Set the arm in a position to draw a shape
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    std::map<std::string, double> joint_targets;
    joint_targets["elbow_joint"] = M_PI_2;
    joint_targets["shoulder_lift_joint"] = -M_PI_2;
    joint_targets["shoulder_pan_joint"] = M_PI_2;
    joint_targets["wrist_1_joint"] = -M_PI;
    joint_targets["wrist_2_joint"] = -M_PI_2;
    joint_targets["wrist_3_joint"] = 0;

    bool joint_plan_success {
        ArmController::planToJointTargets(planning_options, arm_move_group,
                                          joint_plan, joint_targets)};

    if (joint_plan_success) {
        ROS_INFO("Getting ready to draw a square on XZ plane...");
        arm_move_group.execute(joint_plan);
    }

    // Adjust end-effector to the square center to begin drawing at
    auto current_point {arm_move_group.getCurrentPose().pose.position};
    auto adjust_x {center_point.x - current_point.x};
    auto adjust_y {center_point.y - current_point.y};
    auto adjust_z {center_point.z - current_point.z};
    MoveBy(adjust_x, adjust_y, adjust_z, arm_move_group);

    std::string reference_frame {arm_move_group.getPlanningFrame()};
    geometry_msgs::Pose current_pose {arm_move_group.getCurrentPose().pose};

    std::vector<geometry_msgs::Pose> waypoints;
    double radius {(side_length * sqrt(2)) / 2};

    // Generate waypoints for a square trajectory
    for (double theta {0}; theta <= 2*M_PI; theta += M_PI/180) {
        geometry_msgs::Pose point_on_square {current_pose};

        double x_component {std::clamp(radius * cos(theta), -side_length/2, side_length/2)};
        double z_component {std::clamp(radius * sin(theta), -side_length/2, side_length/2)};

        point_on_square.position.x = current_pose.position.x + x_component;
        point_on_square.position.z = current_pose.position.z + z_component;

        waypoints.push_back(point_on_square);
    }

    // Return back to the square center
    waypoints.push_back(current_pose);

    auto square_trajectory {ArmController::planCartesianPath(current_pose,
                                                             waypoints,
                                                             reference_frame,
                                                             arm_move_group)};
    ROS_INFO("Trying to draw a square on XZ plane");

    n.setParam("/record_pose", true);
    arm_move_group.execute(square_trajectory);
    n.setParam("/record_pose", false);
}

int main(int argc, char **argv)
{
     // Setup ROS node
    ros::init(argc, argv, "square");
    ros::AsyncSpinner spinner(4);
    spinner.start();

    // Create PlanningOptions
    MoveitPlanning::PlanningOptions planning_options =
    MoveitPlanning::PlanningOptions();
    planning_options.num_attempts = 10;
    planning_options.allow_replanning = true;
    planning_options.set_planning_time = 30.0;
    planning_options.goal_position_tolerance = 0.01;
    planning_options.goal_orientation_tolerance = 0.01;
    planning_options.goal_joint_tolerance = 0.01;
    planning_options.velocity_scaling_factor = 0.1;
    planning_options.acceleration_scaling_factor = 0.1;

    // Create instance of MoveGroupInterface for given joint group
    moveit::planning_interface::MoveGroupInterface arm_move_group("manipulator");

    // Setup a transform listener to get the pose of the robot base link
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    //Write your code for following the square trajectory here.

    auto robot_position {GetRobotBasePose().translation};

    // XY Plane
    // Find the center and side length to draw the largest square
    auto min_robot_reach_xy {robot_position};
    min_robot_reach_xy.y += ROBOT_MIN_REACH_XY;

    auto max_robot_reach_xy {robot_position};
    max_robot_reach_xy.y += ROBOT_MAX_REACH_XY;

    double work_radius_xy {max_robot_reach_xy.y - min_robot_reach_xy.y};
    double side_length_xy {work_radius_xy * sin(60 * M_PI/180)};

    auto center_point_xy {min_robot_reach_xy};
    center_point_xy.y += side_length_xy / 2;

    // Set the offset height from the table to draw the square
    center_point_xy.z += 0.1;

    ROS_INFO("Computed square side length XY: %f", side_length_xy);
    DrawASquareXY(side_length_xy, center_point_xy, planning_options, arm_move_group);

    // XZ Plane
    // Find the center and side length to draw the largest square
    auto min_robot_reach_xz {robot_position};
    min_robot_reach_xz.z += ROBOT_MIN_REACH_XZ;

    auto max_robot_reach_xz {robot_position};
    max_robot_reach_xz.z += ROBOT_MAX_REACH_XZ;

    double work_radius_xz {max_robot_reach_xz.z - min_robot_reach_xz.z};
    double side_length_xz {work_radius_xz * sin(60 * M_PI/180)};

    auto center_point_xz {min_robot_reach_xz};
    center_point_xz.z += side_length_xz / 2;

    // Set an offset along Y-axis to prevent singularities
    center_point_xz.y += ROBOT_MIN_REACH_XY + 0.1;

    ROS_INFO("Computed square side length XZ: %f", side_length_xz);
    DrawASquareXZ(side_length_xz, center_point_xz, planning_options, arm_move_group);
}
