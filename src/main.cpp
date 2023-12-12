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

ros::Publisher jointsStatePub;

Vector6d readJointsState()
{
    boost::shared_ptr<sensor_msgs::JointState const> jointsStateMessage;
    jointsStateMessage = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    Eigen::VectorXd jointsState(8); 
    for (int i = 0; i < 8; ++i) 
    {
        jointsState(i) = jointsStateMessage->position[i];
    }
    Vector6d joints_arm; 
    joints_arm << jointsState(4), jointsState(3), jointsState(0), jointsState(5), jointsState(6), jointsState(7);
    return joints_arm;
}

void applyMovement(Eigen::Matrix<double, 6, Eigen::Dynamic> mv)
{
    ros::Rate loop_rate(10);
    for (int i = 0; i < mv.cols(); ++i)
    {
        Vector6d q = mv.block(0, i, 6, 1);
        std_msgs::Float64MultiArray jointsStateMessage;
        jointsStateMessage.data.resize(8);
        for (int i = 0; i < 6; i++)
        {
            jointsStateMessage.data[i] = q(i);
        }
        // jointsStateMessage.data[5] = 3.49;
        jointsStateMessage.data[6] = 0.00;
        jointsStateMessage.data[7] = 0.00;
        jointsStatePub.publish(jointsStateMessage);
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_joint_position_publisher");

    ros::NodeHandle nodeHandler;
    jointsStatePub = nodeHandler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    Vector6d q = readJointsState();

    Eigen::Matrix4d initTransormationMatrix = directKin(q);
    Eigen::Vector3d initPoint = initTransormationMatrix.block<3,1>(0,3);
    Eigen::Quaterniond initQuaternion(initTransormationMatrix.block<3,3>(0,0));
    Eigen::Vector3d finalPoint(initPoint(0)+0.2, initPoint(1)+0.1, initPoint(2)-0.1);
    Eigen::Vector3d finalEulerAngles(0.0, 0.0, 0.0); 
    Eigen::Matrix3d finalRotationMatrix = fromEulerAnglesToRotationMatrix(finalEulerAngles);
    Eigen::Quaterniond finalQuaternion(finalRotationMatrix);
    Eigen::Matrix<double, 6, Eigen::Dynamic> movement = inverseKinWithQuaternions(q, initPoint, finalPoint, initQuaternion, finalQuaternion);

    applyMovement(movement);

    q = readJointsState();
    std::cout << "actual direct kinematics = \n" << directKin(q) << std::endl;
    std::cout << "expected point = \n" << finalPoint << std::endl;
    std::cout << "expected rotation matrix = \n" << finalRotationMatrix << std::endl;

    ros::spin();
}
