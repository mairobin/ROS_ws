#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ur_kinematics/ur_kin.h>
#include <vector>

// Define the UR5 robot kinematics and motion control class
class UR5Motion {
public:
    UR5Motion(ros::NodeHandle& nh) {
        // Initialize the publisher for joint trajectory (Correct message type)
        joint_pub_ = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 1);
    }

    // Function to move the robot in joint space
    void moveJointSpace(const double* point1, const double* point2, double velocity, double acceleration) {
        // Here you would call the inverse kinematics solver to calculate joint angles.
        // For now, we'll assume the result is directly available as an array.
        double target_joint_angles[6]; // Assuming 6 joint UR5 robot

        // Inverse kinematics to get joint angles (dummy data here, should be replaced by actual IK solver)
        ur_kinematics::inverse(point1, target_joint_angles); // Replace with actual IK call

        // Create a JointTrajectory message
        trajectory_msgs::JointTrajectory traj_msg;
        
        // Joint names for the UR5 robot (should match your robot's joint names)
        traj_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        // Create a JointTrajectoryPoint to hold the joint positions, velocities, and accelerations
        trajectory_msgs::JointTrajectoryPoint traj_point;
        traj_point.positions = std::vector<double>(target_joint_angles, target_joint_angles + 6);  // Use the target joint angles
        traj_point.velocities = std::vector<double>(6, velocity); // Set velocities (same for all joints)
        traj_point.accelerations = std::vector<double>(6, acceleration); // Set accelerations (same for all joints)

        // Add the trajectory point
        traj_msg.points.push_back(traj_point);

        // Publish the joint trajectory
        joint_pub_.publish(traj_msg);
    }

    // Function to move the robot in Cartesian space
    void moveCartesianSpace(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2, double linear_velocity, double linear_acceleration) {
        // Convert the position from geometry_msgs::Point to a double array
        double point1[3] = {pose1.position.x, pose1.position.y, pose1.position.z};
        double point2[3] = {pose2.position.x, pose2.position.y, pose2.position.z};

        // Call moveJointSpace with the correct arguments
        moveJointSpace(point1, point2, linear_velocity, linear_acceleration);
    }

private:
    ros::Publisher joint_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ur5_kinematics_node");
    ros::NodeHandle nh;

    UR5Motion ur5_motion(nh);

    // Define some test data (example poses in Cartesian space)
    geometry_msgs::Pose pose1, pose2;

    // Set values for pose1 (starting position)
    pose1.position.x = 0.5;
    pose1.position.y = 0.0;
    pose1.position.z = 0.5;

    // Set values for pose2 (target position)
    pose2.position.x = 0.6;
    pose2.position.y = 0.1;
    pose2.position.z = 0.6;

    // Define linear velocity and acceleration
    double linear_velocity = 0.1;  // m/s
    double linear_acceleration = 0.2;  // m/s^2

    // Move the robot in Cartesian space
    ur5_motion.moveCartesianSpace(pose1, pose2, linear_velocity, linear_acceleration);

    ros::spin();
    return 0;
}
