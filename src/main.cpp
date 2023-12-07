/* My lib */
#include "robotic_project/kinematics.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

/* ROS lib */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h" 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_joint_position_publisher");
    ros::NodeHandle nh;

    Eigen::Vector3d P(0.5, -0.5, 0.5);
    Eigen::Matrix3d R {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0}
    };

    UR5 myUR5(nh);
    myUR5.motion_plan(P, R);

    ros::spin();
}
