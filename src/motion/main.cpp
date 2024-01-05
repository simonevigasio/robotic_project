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
            log the location and orientation given from the vision service
        */
        std::cout << srv.response.p.position << std::endl;
        std::cout << "brick yaw angle = " << srv.response.p.orientation.z << std::endl;

        /*
            compute the rotation matrix to get when the robot will grasp the brick
        */
        double robot_grasp_angle = -(srv.response.p.orientation.z + M_PI/2);
        M3d brick_rotation_matrix = rotation_matrix_z_axis(robot_grasp_angle); 

        /*
            compute the position of the brick respect the base frame
        */
        V3d world_brick_position(srv.response.p.position.x, srv.response.p.position.y, srv.response.p.position.z);
        V3d base_brick_position = world_to_base(world_brick_position);

        /*
            compute the final position of the brick
        */
        V3d world_final_destination(0.3, 0.3, 0);
        V3d base_final_destination = world_to_base(world_final_destination);

        /*
            trigger movements
        */
        grasp_and_move_object(base_brick_position, brick_rotation_matrix, base_final_destination, M3d::Identity(), pub);

        /*
            signal of end
        */
        ROS_INFO("COMPLETED");
    }

    ros::spin();
}
