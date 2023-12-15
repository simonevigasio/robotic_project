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

    ros::NodeHandle node_handler;
    ros::Publisher joint_state_pub = node_handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    V8d m = read_robot_measures();
    V6d js = arm_joint_state(m);

    M4d tm = direct_kin(js);
    M3d rm = tm.block(0, 0, 3, 3);
    V3d p_i = tm.block(0, 3, 3, 1);

    V3d p_f_w {0.64, 0.58, 0.86};
    V3d p_f = world_to_base(p_f_w);
    p_f(2) = 0.5;

    Qd q_i(rm), q_f(M3d::Identity());

    Path pt = differential_inverse_kin_quaternions(m, p_i, p_f, q_i, q_f);
    apply_movement(pt, joint_state_pub);

    m = read_robot_measures();
    js = arm_joint_state(m);

    std::cout << "expected point = \n" << p_f << std::endl;
    std::cout << "actual direct = \n" << direct_kin(js) << std::endl;

    ros::spin();
}
