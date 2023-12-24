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
        V3d world_brick_position;
        world_brick_position << srv.response.p.position.x, srv.response.p.position.y, srv.response.p.position.z;
        world_brick_position << 0.3, 0.3, 0.5;
        V3d base_brick_position = world_to_base(world_brick_position);
        base_brick_position(2) = 0.5;

        // V6d joint_state = get_joint_state(read_robot_measures());
        // M4d t = direct_kin(joint_state);
        // V3d p = t.block(0, 3, 3, 1);
        // M3d r = t.block(0, 0, 3, 3);
        // Qd q(r), fq(M3d::Identity());

        // Path path = differential_inverse_kin_quaternions(read_robot_measures(), p, base_brick_position, q, fq);
        // move(path, pub);

        // move_end_effector(base_brick_position, M3d::Identity(), pub);

        std::cout << read_robot_measures() << std::endl;

        ROS_INFO("end");
    }

    ros::spin();
}
