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
    // ROS init
    ros::init(argc, argv, "ur5_joint_position_publisher");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    Vector6d q;

    try 
    {
        q = read_q();
    }
    catch(...)
    {
        std::cout << "exception accured" << std::endl;
        exit(1);
    }

    Eigen::Matrix4d initial_transormation_matrix = direct_kinematics(q);
    Eigen::Vector3d initial_point = initial_transormation_matrix.block<3,1>(0,3);
    Eigen::Quaterniond initial_quaternion(initial_transormation_matrix.block<3,3>(0,0));
    Eigen::Vector3d initial_euler_angles = from_rotation_matrix_to_euler_angles(initial_transormation_matrix.block<3,3>(0,0));

    Eigen::Vector3d final_point(initial_point(0)+0.2, initial_point(1), initial_point(2));
    Eigen::Vector3d final_euler_angles(0.0, 0.0, 0.0); 
    Eigen::Matrix3d final_rotation_matrix = from_euler_angles_to_rotation_matrix(final_euler_angles);
    Eigen::Quaterniond final_quaternion(final_rotation_matrix);

    std::cout << "Final position" << std::endl;
    std::cout << final_point << std::endl;

    Eigen::Matrix<double, 6, Eigen::Dynamic> trajectory = inverse_differential_kinematics(q, initial_point, initial_euler_angles, final_point, final_euler_angles);
//    Eigen::Matrix<double, 6, Eigen::Dynamic> trajectory = inverse_differential_kinematics_with_quaternions(q, initial_point, initial_quaternion, final_point, final_quaternion);

    ros::Rate loop_rate(125);

    for (int i=0; i<trajectory.cols(); ++i)
    {
        Vector6d move = trajectory.block<6,1>(0,i); 
        std_msgs::Float64MultiArray msg;
        msg.data.clear();
        msg.data = {move(0), move(1), move(2), move(3), move(4), move(5), 0.0, 0.0};
        std::cout << "Dati" << std::endl;
        std::cout << move << std::endl;
        pub.publish(msg);
        loop_rate.sleep();
    }

    // ROS spin
    ros::spin();
}
