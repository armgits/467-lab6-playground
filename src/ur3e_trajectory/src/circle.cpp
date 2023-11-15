#include "../include/circle.hpp"

bool MoveBy(double x, double y, double z,
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

    return true;
}

void DrawACircleXY(double radius,
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
        ROS_INFO("Getting ready to draw a circle on XY plane...");
        arm_move_group.execute(joint_plan);
    }

    // Adjust end-effector to the circle center to begin drawing at
    MoveBy(0.125, 0.1, -0.2, arm_move_group);

    std::string reference_frame {arm_move_group.getPlanningFrame()};
    geometry_msgs::Pose current_pose {arm_move_group.getCurrentPose().pose};

    std::vector<geometry_msgs::Pose> waypoints;

    // Generate waypoints for a circular trajectory
    for (double theta {0}; theta <= 2*M_PI; theta += M_PI/180) {
        geometry_msgs::Pose point_on_circle {current_pose};
        point_on_circle.position.x = current_pose.position.x + (radius * cos(theta));
        point_on_circle.position.y = current_pose.position.y + (radius * sin(theta));

        waypoints.push_back(point_on_circle);
    }

    // Return back to the circle center
    waypoints.push_back(current_pose);

    auto circular_trajectory {ArmController::planCartesianPath(current_pose,
                                                               waypoints,
                                                               reference_frame,
                                                               arm_move_group)};

    ROS_INFO("Trying to draw a circle on XY plane...");

    n.setParam("/record_pose", true);
    arm_move_group.execute(circular_trajectory);
    n.setParam("/record_pose", false);
}

void DrawACircleXZ(double radius,
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
        ROS_INFO("Getting ready to draw a circle on XZ plane...");
        arm_move_group.execute(joint_plan);
    }

    // Adjust end-effector to the circle center to begin drawing at
    MoveBy(0.125, 0.1, -0.13, arm_move_group);

    std::string reference_frame {arm_move_group.getPlanningFrame()};
    geometry_msgs::Pose current_pose {arm_move_group.getCurrentPose().pose};

    std::vector<geometry_msgs::Pose> waypoints;

    // Generate waypoints for a circular trajectory
    for (double theta {0}; theta <= 2*M_PI; theta += M_PI/180) {
        geometry_msgs::Pose point_on_circle {current_pose};
        point_on_circle.position.x = current_pose.position.x + (radius * cos(theta));
        point_on_circle.position.z = current_pose.position.z + (radius * sin(theta));

        waypoints.push_back(point_on_circle);
    }

    // Return back to the circle center
    waypoints.push_back(current_pose);

    auto circular_trajectory {ArmController::planCartesianPath(current_pose,
                                                               waypoints,
                                                               reference_frame,
                                                               arm_move_group)};

    ROS_INFO("Trying to draw a circle on XZ plane...");

    n.setParam("/record_pose", true);
    arm_move_group.execute(circular_trajectory);
    n.setParam("/record_pose", false);
}

int main(int argc, char **argv)
{
    // Setup ROS node
    ros::init(argc, argv, "circle");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::NodeHandle n;

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

    //Write your code for following the circle trajectory here.

    // XY Plane
    DrawACircleXY(0.125, planning_options, arm_move_group);

    // XZ Plane
    DrawACircleXZ(0.15, planning_options, arm_move_group);

}
