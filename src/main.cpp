// Header
#include "robotic_project/kinematics.h"

// Eigen
#include "Eigen/Dense"

// Standard
#include <iostream>
#include <cmath>

// ROS
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h" 

int main(int argc, char **argv)
{
    // ROS init
    ros::init(argc, argv, "ur5_joint_position_publisher");

    // UR5 movement 
    UR5 UR5obj;   

    // ROS spin
    ros::spin();
}
