// Header
#include "robotic_project/kinematics.h"

// Eigen
#include "Eigen/Dense"

// Standard
#include <iostream>
#include <cmath>

// ROS
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "boost/shared_ptr.hpp"

ros::Publisher joints_state_pub;

Vector6d read_q()
{
    boost::shared_ptr<sensor_msgs::JointState const> joint_state_message;
    joint_state_message = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    Eigen::VectorXd joints_state(8); 
    for (int i = 0; i < 8; ++i) 
    {
        joints_state(i) = joint_state_message->position[i];
    }
    Vector6d joints_arm; 
    joints_arm << joints_state(4), joints_state(3), joints_state(0), joints_state(5), joints_state(6), joints_state(7);
    return joints_arm;
}

void setJointsState(Vector6d q)
{
    ros::Rate loop_rate(10);
    std_msgs::Float64MultiArray joint_state_message;
    joint_state_message.data.resize(8);
    for (int i = 0; i < 6; i++)
    {
        joint_state_message.data[i] = q(i);
    }
    joint_state_message.data[5] = 3.49;
    joint_state_message.data[6] = 0.00;
    joint_state_message.data[7] = 0.00;
    joints_state_pub.publish(joint_state_message);
    loop_rate.sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_joint_position_publisher");

    ros::NodeHandle nodeHandler;
    joints_state_pub = nodeHandler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    Vector6d q = read_q();

    Eigen::Matrix4d initTransormationMatrix = direct_kinematics(q);
    Eigen::Vector3d initPoint = initTransormationMatrix.block<3,1>(0,3);
    Eigen::Quaterniond initQuaternion(initTransormationMatrix.block<3,3>(0,0));
    Eigen::Vector3d finalPoint(initPoint(0)-0.5, initPoint(1), initPoint(2));
    Eigen::Vector3d finalEulerAngles(0.0, 0.0, 0.0); 
    Eigen::Matrix3d finalRotationMatrix = from_euler_angles_to_rotation_matrix(finalEulerAngles);
    Eigen::Quaterniond finalQuaternion(finalRotationMatrix);

    std::cout << "Final position" << std::endl;
    std::cout << finalPoint << std::endl;

    Eigen::Matrix<double, 6, Eigen::Dynamic> trajectory = inverse_differential_kinematics_with_quaternions(q, initPoint, initQuaternion, finalPoint, finalQuaternion);

    ros::Rate loop_rate(10);

    for (int i=0; i<trajectory.cols(); ++i)
    {
        Vector6d move = trajectory.block<6,1>(0,i); 
        std_msgs::Float64MultiArray msg;
        msg.data.clear();
        msg.data = {move(0), move(1), move(2), move(3), move(4), move(5), 0.0, 0.0};
        joints_state_pub.publish(msg);
        loop_rate.sleep();
    }

    ros::spin();
}
