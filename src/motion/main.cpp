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
    
    /*
        call vision service to obtain the position and orientation of the brick
    */
    ros::ServiceClient service_client = node_handler.serviceClient<robotic_project::ObtainBrickPose>("obtain_brick_pose");
    robotic_project::ObtainBrickPose srv;
    if (service_client.call(srv))
    {
        /*
            log the location given from the vision service
        */
        std::cout << srv.response.p.position << std::endl;

        /* 
            open gripper
        */
        toggle_gripper(pub, true);

        /*
            compute the point based on the base frame
        */
        V3d wp;
        wp << srv.response.p.position.x, srv.response.p.position.y, srv.response.p.position.z;
        V3d bp = world_to_base(wp);

        /*
            place the arm above the brick
        */
        bp(2) = 0.50;
        move_end_effector(bp, V3d::Zero(), pub);

        /*
            move downwards to grasp the brick
        */
        bp(2) = 0.72;
        move_end_effector(bp, V3d::Zero(), pub);

        /*
            close gripper
        */
        toggle_gripper(pub);

        /*
            move upwards
        */
        bp(2) = 0.50;
        move_end_effector(bp, V3d::Zero(), pub);

        /*
            move 30cm left respect the robot
        */
        bp(0) = bp(0) - 0.30;
        move_end_effector(bp, V3d::Zero(), pub);

        /*
            move downwards to leave the brick
        */
        bp(2) = 0.72;
        move_end_effector(bp, V3d::Zero(), pub);

        /*
            open gripper
        */
        toggle_gripper(pub);

        /*
            move upwards
        */
        bp(2) = 0.50;
        move_end_effector(bp, V3d::Zero(), pub);

        /*
            move 30cm right respect the robot
        */
        bp(0) = bp(0) + 0.30;
        move_end_effector(bp, V3d::Zero(), pub);

        /*
            close gripper
        */
        toggle_gripper(pub);
    }

    ros::spin();
}
