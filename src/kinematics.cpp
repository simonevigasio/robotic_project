// Header
#include "robotic_project/kinematics.h"

// ROS
#include "std_msgs/Float64MultiArray.h" 
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

// Eigen 
#include "Eigen/Dense"

// Standard 
#include <iostream>
#include <cmath>

// Constructor
UR5::UR5() 
{
    __a << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
    __d << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
    __alpha << M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0;

    __a *= __scalar_factor;
    __d *= __scalar_factor;

    __sub = __nh.subscribe("/ur5/joint_states", 10, &UR5::handle_movement, this);
    __pub = __nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);

    __Kp = Eigen::Matrix3d::Identity()*10; 
    __Kq = Eigen::Matrix3d::Identity()*-10; 

    __final_quaternion = from_rotational_matrix_to_quaternion(Eigen::Matrix3d::Identity()); 
    __final_position << 1.5, -1.9, 4.5; std::cout << "final position = " << std::endl << __final_position << std::endl;
}

// Callback of the subscriber
void UR5::handle_movement(const sensor_msgs::JointState& msg) 
{
    // Get theta angles 
    Eigen::VectorXd q(8);
    for (int i=0; i<8; ++i) q(i) = msg.position[i];
    __q << q(4), q(3), q(0), q(5), q(6), q(7);

    // Update the position
    direct_kinematics();
    compute_geometric_jacobian(); 

    if (!__init)
    {
        // Set start of the trajectory and the destination 
        __time_trajectory = __delta_time;
        __initial_position = __pe;
        __initial_quaternion = from_rotational_matrix_to_quaternion(__Re);
        __init = !__init;
    }
    else 
    {   
        // Next joint values 
        std_msgs::Float64MultiArray jointCommand;
        if (__time_trajectory <= __duration_trajectory-__delta_time && __time_trajectory >= __delta_time)
        {
            //std::cout << "current position = " << std::endl << __pe << std::endl;

            // Nomalization of the time trajectory 
            double nomalized_time = __time_trajectory/__duration_trajectory;
            double previous_normalized_time = (__time_trajectory-__delta_time)/__duration_trajectory;
            double next_normalized_time = (__time_trajectory+__delta_time)/__duration_trajectory;

            // Desired position and quaternian
            Eigen::Vector3d desired_position = nomalized_time*__final_position+(1-nomalized_time)*__initial_position; 
            //std::cout << "desired position = " << std::endl << desired_position << std::endl;
            Eigen::Quaterniond desired_quaternion = __initial_quaternion.slerp(nomalized_time, __final_quaternion);

            // Desired previous position 
            Eigen::Vector3d desired_previous_position = previous_normalized_time*__final_position+(1-previous_normalized_time)*__initial_position; 
            //std::cout << "desired previous position = " << std::endl << desired_previous_position << std::endl;

            // Desired next quaternian
            Eigen::Quaterniond desired_next_quaternion = __initial_quaternion.slerp(next_normalized_time, __final_quaternion);

            // End-effector velocity 
            Eigen::Vector3d end_effector_velocity = (desired_position-desired_previous_position)/__delta_time; 
            //std::cout << "end effector velocity = " << std::endl << end_effector_velocity << std::endl;

            // Angular velocity
            Eigen::Vector3d angular_velocity = (__delta_time/2)*(desired_next_quaternion*desired_quaternion.conjugate()).vec();

            // Position error
            Eigen::Vector3d position_error = desired_position-__pe; 
            std::cout << "position error = " << std::endl << position_error << std::endl;

            // Quaternion error
            Eigen::Quaterniond quaternion_error = desired_quaternion*from_rotational_matrix_to_quaternion(__Re).conjugate(); 

            // Next Joints 
            Eigen::VectorXd next_joints = __q+joints_velocity(end_effector_velocity, position_error, angular_velocity, quaternion_error)*__delta_time;
            jointCommand.data = {next_joints(0), next_joints(1), next_joints(2), next_joints(3), next_joints(4), next_joints(5), 0.0, 0.0};
            //std::cout << "next joints = " << std::endl << next_joints << std::endl;

            // Increment time trajectory
            __time_trajectory += __delta_time; 
            std::cout << "time = " << std::endl << __time_trajectory << std::endl;
        }
        else jointCommand.data = {__q(0), __q(1), __q(2), __q(3), __q(4), __q(5), 0.0, 0.0};
        __pub.publish(jointCommand);
    }
}

// Direct Kinematics 
void UR5::direct_kinematics() 
{
    Eigen::Matrix4d T10 = generate_transformation_matrix(__a(0), __alpha(0), __d(0), __q(0));
    Eigen::Matrix4d T21 = generate_transformation_matrix(__a(1), __alpha(1), __d(1), __q(1));
    Eigen::Matrix4d T32 = generate_transformation_matrix(__a(2), __alpha(2), __d(2), __q(2));
    Eigen::Matrix4d T43 = generate_transformation_matrix(__a(3), __alpha(3), __d(3), __q(3));
    Eigen::Matrix4d T54 = generate_transformation_matrix(__a(4), __alpha(4), __d(4), __q(4));
    Eigen::Matrix4d T65 = generate_transformation_matrix(__a(5), __alpha(5), __d(5), __q(5));
    Eigen::Matrix4d T60 = T10*T21*T32*T43*T54*T65;
    
    __pe=T60.block<3,1>(0,3);
    __Re=T60.block<3,3>(0,0);
}

// Generate T from i-1 to i given the DH parameters 
Eigen::Matrix4d UR5::generate_transformation_matrix(double a, double alpha, double d, double theta)
{
    return Eigen::Matrix4d {
        {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)},
        {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)},
        {0, sin(alpha), cos(alpha), d},
        {0, 0, 0, 1}
    };
}

// Geometric jacobian and inverse
void UR5::compute_geometric_jacobian()
{
    Vector6d J1(
        __d(4)*(cos(__q(0))*cos(__q(4)) + cos(__q(1) + __q(2) + __q(3))*sin(__q(0))*sin(__q(4))) + __d(3)*cos(__q(0)) - __a(1)*cos(__q(1))*sin(__q(0)) - __d(4)*sin(__q(1) + __q(2) + __q(3))*sin(__q(0)) - __a(2)*cos(__q(1))*cos(__q(2))*sin(__q(0)) + __a(2)*sin(__q(0))*sin(__q(1))*sin(__q(2)),
        __d(4)*(cos(__q(4))*sin(__q(0)) - cos(__q(1) + __q(2) + __q(3))*cos(__q(0))*sin(__q(4))) + __d(3)*sin(__q(0)) + __a(1)*cos(__q(0))*cos(__q(1)) + __d(4)*sin(__q(1) + __q(2) + __q(3))*cos(__q(0)) + __a(2)*cos(__q(0))*cos(__q(1))*cos(__q(2)) - __a(2)*cos(__q(0))*sin(__q(1))*sin(__q(2)),
        0,
        0,
        0,
        1
    );

    Vector6d J2(
        -cos(__q(0))*(__a(2)*sin(__q(1) + __q(2)) + __a(1)*sin(__q(1)) + __d(4)*(sin(__q(1) + __q(2))*sin(__q(3)) - cos(__q(1) + __q(2))*cos(__q(3))) - __d(4)*sin(__q(4))*(cos(__q(1) + __q(2))*sin(__q(3)) + sin(__q(1) + __q(2))*cos(__q(3)))),
        -sin(__q(0))*(__a(2)*sin(__q(1) + __q(2)) + __a(1)*sin(__q(1)) + __d(4)*(sin(__q(1) + __q(2))*sin(__q(3)) - cos(__q(1) + __q(2))*cos(__q(3))) - __d(4)*sin(__q(4))*(cos(__q(1) + __q(2))*sin(__q(3)) + sin(__q(1) + __q(2))*cos(__q(3)))),
        __a(2)*cos(__q(1) + __q(2)) - (__d(4)*sin(__q(1) + __q(2) + __q(3) + __q(4)))/2 + __a(1)*cos(__q(1)) + (__d(4)*sin(__q(1) + __q(2) + __q(3) - __q(4)))/2 + __d(4)*sin(__q(1) + __q(2) + __q(3)),
        sin(__q(0)),
        -cos(__q(0)),
        0
    );

    Vector6d J3(
        cos(__q(0))*(__d(4)*cos(__q(1) + __q(2) + __q(3)) - __a(2)*sin(__q(1) + __q(2)) + __d(4)*sin(__q(1) + __q(2) + __q(3))*sin(__q(4))),
        sin(__q(0))*(__d(4)*cos(__q(1) + __q(2) + __q(3)) - __a(2)*sin(__q(1) + __q(2)) + __d(4)*sin(__q(1) + __q(2) + __q(3))*sin(__q(4))),
        __a(2)*cos(__q(1) + __q(2)) - (__d(4)*sin(__q(1) + __q(2) + __q(3) + __q(4)))/2 + (__d(4)*sin(__q(1) + __q(2) + __q(3) - __q(4)))/2 + __d(4)*sin(__q(1) + __q(2) + __q(3)),
        sin(__q(0)),
        -cos(__q(0)),
        0
    );

    Vector6d J4(
        __d(4)*cos(__q(0))*(cos(__q(1) + __q(2) + __q(3)) + sin(__q(1) + __q(2) + __q(3))*sin(__q(4))),
        __d(4)*sin(__q(0))*(cos(__q(1) + __q(2) + __q(3)) + sin(__q(1) + __q(2) + __q(3))*sin(__q(4))),
        __d(4)*(sin(__q(1) + __q(2) + __q(3) - __q(4))/2 + sin(__q(1) + __q(2) + __q(3)) - sin(__q(1) + __q(2) + __q(3) + __q(4))/2),
        sin(__q(0)),
        -cos(__q(0)),
        0
    );

    Vector6d J5(
        __d(4)*cos(__q(0))*cos(__q(1))*cos(__q(4))*sin(__q(2))*sin(__q(3)) - __d(4)*cos(__q(0))*cos(__q(1))*cos(__q(2))*cos(__q(3))*cos(__q(4)) - __d(4)*sin(__q(0))*sin(__q(4)) + __d(4)*cos(__q(0))*cos(__q(2))*cos(__q(4))*sin(__q(1))*sin(__q(3)) + __d(4)*cos(__q(0))*cos(__q(3))*cos(__q(4))*sin(__q(1))*sin(__q(2)),
        __d(4)*cos(__q(0))*sin(__q(4)) + __d(4)*cos(__q(1))*cos(__q(4))*sin(__q(0))*sin(__q(2))*sin(__q(3)) + __d(4)*cos(__q(2))*cos(__q(4))*sin(__q(0))*sin(__q(1))*sin(__q(3)) + __d(4)*cos(__q(3))*cos(__q(4))*sin(__q(0))*sin(__q(1))*sin(__q(2)) - __d(4)*cos(__q(1))*cos(__q(2))*cos(__q(3))*cos(__q(4))*sin(__q(0)),
        -__d(4)*(sin(__q(1) + __q(2) + __q(3) - __q(4))/2 + sin(__q(1) + __q(2) + __q(3) + __q(4))/2),
        sin(__q(1) + __q(2) + __q(3))*cos(__q(0)),
        sin(__q(1) + __q(2) + __q(3))*sin(__q(0)),
        -cos(__q(1) + __q(2) + __q(3))
    );

    Vector6d J6(
        0,
        0,
        0,
        cos(__q(4))*sin(__q(0)) - cos(__q(1) + __q(2) + __q(3))*cos(__q(0))*sin(__q(4)),
        -cos(__q(0))*cos(__q(4)) - cos(__q(1) + __q(2) + __q(3))*sin(__q(0))*sin(__q(4)),
        -sin(__q(1) + __q(2) + __q(3))*sin(__q(4))   
    );

    __geometric_jacobian.col(0) = J1;
    __geometric_jacobian.col(1) = J2;
    __geometric_jacobian.col(2) = J3;
    __geometric_jacobian.col(3) = J4;
    __geometric_jacobian.col(4) = J5;
    __geometric_jacobian.col(5) = J6;

    __inverse_geometric_jacobian = __geometric_jacobian.inverse();
}

// Generate the quaternian from the rotation matrix
Eigen::Quaterniond UR5::from_rotational_matrix_to_quaternion(Eigen::Matrix3d rotation_matrix)
{
    Eigen::Quaterniond quaternion(rotation_matrix);
	return quaternion;
}

// Trajectory joints velocity 
Eigen::VectorXd UR5::joints_velocity(
    Eigen::Vector3d position_velocity, 
    Eigen::Vector3d position_error, 
    Eigen::Vector3d angular_velocity, 
    Eigen::Quaterniond quaternion_error
) 
{
    Eigen::Vector3d cartesian_input = position_velocity+__Kp*position_error; 
    Eigen::Vector3d quaternion_input = angular_velocity+__Kq*quaternion_error.vec(); 
    Eigen::VectorXd jacobin_input(6); jacobin_input << cartesian_input, quaternion_input;
    Eigen::VectorXd joints_velocity(6); joints_velocity = __inverse_geometric_jacobian*jacobin_input;
    //std::cout << "inverse jacobian = " << std::endl << __inverse_geometric_jacobian << std::endl;
    //std::cout << "joints velocity = " << std::endl << joints_velocity << std::endl;
    return joints_velocity; 
}
