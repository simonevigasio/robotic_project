// Header
#include "robotic_project/kinematics.h"

// Eigen
#include "Eigen/Dense"

// Standard
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <string.h>

// ROS
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "robotic_project/ObtainBrickPose.h"

V2d brick_final_position(std::string brick_type)
{
    if (brick_type == "X1-Y1-Z2") return V2d {0.2, 0.7};
    if (brick_type == "X1-Y2-Z1") return V2d {0.6, 0.7};
    if (brick_type == "X1-Y2-Z2") return V2d {0.4, 0.7};
    if (brick_type == "X1-Y2-Z2-CHAMFER") return V2d {0.2, 0.6};
    if (brick_type == "X1-Y2_Z2-TWINFILLET") return V2d {0.3, 0.6};
    if (brick_type == "X1-Y3-Z2") return V2d {0.4, 0.6};
    if (brick_type == "X1-Y3-Z2-FILLET") return V2d {0.2, 0.5};
    if (brick_type == "X1-Y4-Z1") return V2d {0.2, 0.4};
    if (brick_type == "X1-Y4-Z2") return V2d {0.2, 0.3};
    ROS_ERROR("brick not defined");
    exit(1);
}

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
        std::vector<bool> valid_index;
        for (int i = 0; i < srv.response.length; ++i) valid_index.push_back(true);

        /*
            log the location and orientation given from the vision service
        */
        for (int i = 0; i < srv.response.length; ++i)
        {
            std::cout << "localization brick " << i +1 << ":\n" << srv.response.p[i].position;
            std::cout << "orientation brick " << i + 1 << ":\n" << srv.response.p[i].orientation.z << std::endl;
        }
        
        for (int i = 0; i < srv.response.length; ++i)
        {
            if (valid_index[i])
            {
                /*
                    compute the rotation matrix to get when the robot will grasp the brick
                */
                double robot_grasp_angle = -(srv.response.p[i].orientation.z + M_PI/2);
                M3d brick_rotation_matrix = rotation_matrix_z_axis(robot_grasp_angle); 

                /*
                    compute the position of the brick respect the base frame
                */
                std::cout << "going to grasp brick in location\n" << srv.response.p[i].position << std::endl;
                std::cout << "brick name = " << srv.response.name[i] << std::endl;
                V3d world_brick_position(srv.response.p[i].position.x, srv.response.p[i].position.y, srv.response.p[i].position.z);
                V3d base_brick_position = world_to_base(world_brick_position);

                /*
                    compute the final position of the brick
                */
                V2d final_position = brick_final_position(srv.response.name[i]);
                V3d world_final_destination(final_position(0), final_position(1), 0);
                V3d base_final_destination = world_to_base(world_final_destination);

                /*
                    trigger movements
                */
                grasp_and_move_object(base_brick_position, brick_rotation_matrix, base_final_destination, M3d::Identity(), pub);
            }
        }
        
        /*
            signal of end
        */
        ROS_INFO("COMPLETED");
    }

    ros::spin();
}
