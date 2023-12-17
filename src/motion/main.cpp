// Header
#include "robotic_project/kinematics.h"

// Eigen
#include "Eigen/Dense"

// Standard
#include <iostream>
#include <cstdlib>
#include <cmath>

// ROS
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "robotic_project/ObtainBrickPose.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_joint_position_publisher");

    ros::NodeHandle node_handler;
    ros::Publisher pub = node_handler.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    // Service 

    ros::ServiceClient service_client = node_handler.serviceClient<robotic_project::ObtainBrickPose>("obtain_brick_pose");

    robotic_project::ObtainBrickPose srv;

    if (service_client.call(srv))
    {
        std::cout << srv.response.p.position << std::endl;

        V3d p_f_w {srv.response.p.position.x, srv.response.p.position.y, srv.response.p.position.z};
        V3d p_f = world_to_base(p_f_w);
        p_f(2) = 0.5;

        move(p_f, V3d::Identity(), pub);

        V8d m = read_robot_measures();
        V6d js = arm_joint_state(m);

        std::cout << "expected point = \n" << p_f << std::endl;
        std::cout << "actual direct = \n" << direct_kin(js) << std::endl;

        Path pt = open_gripper(m);
        apply_movement(pt, pub);
        m = read_robot_measures();

        M4d tm = direct_kin(js);
        M3d rm = tm.block(0, 0, 3, 3);
        V3d p_i = tm.block(0, 3, 3, 1);

        p_f << p_i(0), p_i(1), 0.72;
        Qd q_i(rm); 
        Qd q_f = q_i;

        pt = differential_inverse_kin_quaternions(m, p_i, p_f, q_i, q_f);
        apply_movement(pt, pub);
        m = read_robot_measures();

        std::cout << "m = \n" << m << std::endl;

        pt = close_gripper(m);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        std::cout << "m = \n" << m << std::endl;

        tm = direct_kin(js);
        rm = tm.block(0, 0, 3, 3);
        p_i = tm.block(0, 3, 3, 1);

        p_f << p_i(0), p_i(1), 0.50;
        q_i = rm; 

        pt = differential_inverse_kin_quaternions(m, p_i, p_f, q_i, q_f);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        std::cout << "m = \n" << m << std::endl;

        tm = direct_kin(js);
        rm = tm.block(0, 0, 3, 3);
        p_i = tm.block(0, 3, 3, 1);

        p_f << p_i(0) - 0.3, p_i(1), p_i(2);
        q_i = rm;

        pt = differential_inverse_kin_quaternions(m, p_i, p_f, q_i, q_f);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        std::cout << "m = \n" << m << std::endl;

        tm = direct_kin(js);
        rm = tm.block(0, 0, 3, 3);
        p_i = tm.block(0, 3, 3, 1);

        p_f << p_i(0), p_i(1), 0.72;
        q_i = rm;

        pt = differential_inverse_kin_quaternions(m, p_i, p_f, q_i, q_f);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        std::cout << "m = \n" << m << std::endl;

        pt = open_gripper(m);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        tm = direct_kin(js);
        rm = tm.block(0, 0, 3, 3);
        p_i = tm.block(0, 3, 3, 1);

        p_f << p_i(0), p_i(1), 0.50;
        q_i = rm;

        pt = differential_inverse_kin_quaternions(m, p_i, p_f, q_i, q_f);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        std::cout << "m = \n" << m << std::endl;

        pt = close_gripper(m);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        std::cout << "m = \n" << m << std::endl;

        tm = direct_kin(js);
        rm = tm.block(0, 0, 3, 3);
        p_i = tm.block(0, 3, 3, 1);

        p_f << p_i(0) + 0.3, p_i(1), p_i(2);
        q_i = rm;

        pt = differential_inverse_kin_quaternions(m, p_i, p_f, q_i, q_f);
        apply_movement(pt, pub);
        m = read_robot_measures();
        js = arm_joint_state(m);

        std::cout << "m = \n" << m << std::endl;

        std::cout << "END" << std::endl;
    }

   

    ros::spin();
}
