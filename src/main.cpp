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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_joint_position_publisher");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    Vector6d q;

    try 
    {
        q = read_q();
        std::cout << "q =\n" << q << std::endl;

    }
    catch(...)
    {
        std::cout << "exception occured" << std::endl;
    }

    ros::spin();
}
