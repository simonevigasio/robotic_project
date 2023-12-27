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
        std::cout << "orientation z = " << srv.response.p.orientation.z << std::endl;

        toggle_gripper(pub, true);

        M3d brick_rotation_matrix = rotation_matrix_z_axis(-srv.response.p.orientation.z); 
        V3d world_brick_position(srv.response.p.position.x, srv.response.p.position.y, srv.response.p.position.z);
        V3d base_brick_position = world_to_base(world_brick_position);

        set_safe_configuration(pub);

        base_brick_position(2) = 0.5;
        Trajectory tr = build_trajectory(base_brick_position);
        std::cout << tr << std::endl;
        for (int i = 0; i < tr.rows(); ++i)
        {   
            V6d js = get_joint_state(read_robot_measures());
            M4d t = direct_kin(js);
            M3d r; if (i != tr.rows() - 1) r = t.block(0, 0, 3, 3); else r = brick_rotation_matrix;
            move_end_effector(tr.row(i), r, pub);
        }

        V6d js = get_joint_state(read_robot_measures());
        M4d t = direct_kin(js);
        std::cout << t << std::endl;

        base_brick_position(2) = 0.72;
        move_end_effector(base_brick_position, brick_rotation_matrix, pub);
        toggle_gripper(pub);
        base_brick_position(2) = 0.5;
        move_end_effector(base_brick_position, brick_rotation_matrix, pub);

        set_safe_configuration(pub);

        V3d w_final_dest(0.3, 0.5, 0);
        V3d b_final_dest = world_to_base(w_final_dest);
        b_final_dest(2) = 0.5;

        tr = build_trajectory(b_final_dest);
        std::cout << tr << std::endl;

        for (int i = 0; i < tr.rows(); ++i)
        {
            V6d js = get_joint_state(read_robot_measures());
            M4d t = direct_kin(js);
            M3d r = M3d::Identity();
            move_end_effector(tr.row(i), r, pub);
        }

        b_final_dest(2) = 0.72;
        move_end_effector(b_final_dest, M3d::Identity(), pub);
        toggle_gripper(pub);
        b_final_dest(2) = 0.5;
        move_end_effector(b_final_dest, M3d::Identity(), pub);

        ROS_INFO("end");
    }

    ros::spin();
}
