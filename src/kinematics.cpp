#include "robotic_project/kinematics.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h" 
#include "sensor_msgs/JointState.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

UR5::UR5(ros::NodeHandle nh) 
{
    /* default angles setup */
    __q__ <<  -0.32,-0.78, -2.56,-1.63, -1.57, 3.49;

     /* set DH params */
    init_params();

    /* set new position */
    update_position();

    /* setup ROS eviroment */
    init_ROS(nh);
}

/* Constructor */
UR5::UR5(Vector6d q, ros::NodeHandle nh)
{
    /* set qes */
    __q__ = q;

     /* set DH params */
    init_params();

    /* set new position */
    update_position();

    /* setup ROS eviroment */
    init_ROS(nh);
}

void UR5::init_ROS(ros::NodeHandle nh)
{
    /* initialize the subscriber */
    sub = nh.subscribe("/ur5/joint_states", 1, &UR5::update_q, this);

    /* initialize the publisher */
    pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
}

/* set standard DH params */
void UR5::init_params()
{
    /* UR5 assignments for DH parameters */
    a << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
    d << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
    alpha << M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0;
}

/* update the jacobian, __Pe__ and __Re__ */
void UR5::update_position()
{
    /* Compute the initial __Pe__ and __Re__ */
    direct_kinematics();

    /* Computer the geometric jacobian */
    compute_geometric_jacobian(); 
}

/* change the value of __q__  and reset the position data */
void UR5::update_q(const sensor_msgs::JointState msg) 
{
    /* angles values */
    Eigen::VectorXd q(8);
    for (int i=0; i<8; ++i) q(i)=msg.position[i];

    /* arm theta angles => the arrangement of the theta angles are given by the subscriber information sequence */
    __q__ << q(4), q(3), q(0), q(5), q(6), q(7);

    /* update new position */
    update_position();

    // try this way
}

/* direct Kinematic computation */
void UR5::direct_kinematics() 
{
    /* calculate  transformation matrix from frame 6 to 0 */
    Eigen::Matrix4d __transformation_matrix__ = generate_transformation_matrix(a(0), alpha(0), d(0), __q__(0));
    for (int i=1; i<6; ++i) __transformation_matrix__ = __transformation_matrix__ * generate_transformation_matrix(a(i), alpha(i), d(i), __q__(i));
    
    /* extract from the transformation matrix the coordinates of the end-effector */
    __pe__=__transformation_matrix__.block<3,1>(0,3);

    /* extract from the transformation matrix the rotation matrix */
    __Re__=__transformation_matrix__.block<3,3>(0,0);
}

/* compute the transformation matrix from frame i to frame i-1 */
Eigen::Matrix4d UR5::generate_transformation_matrix(double a, double alpha, double d, double theta)
{
    return Eigen::Matrix4d {
        {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)},
        {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)},
        {0, sin(alpha), cos(alpha), d},
        {0, 0, 0, 1}
    };
}

/* compute the geometric jacobian */
void UR5::compute_geometric_jacobian()
{
    /* first column of the geometric jacobian */
    Vector6d J1(
        d(4)*(cos(__q__(0))*cos(__q__(4)) + cos(__q__(1) + __q__(2) + __q__(3))*sin(__q__(0))*sin(__q__(4))) + d(3)*cos(__q__(0)) - a(1)*cos(__q__(1))*sin(__q__(0)) - d(4)*sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(0)) - a(2)*cos(__q__(1))*cos(__q__(2))*sin(__q__(0)) + a(2)*sin(__q__(0))*sin(__q__(1))*sin(__q__(2)),
        d(4)*(cos(__q__(0))*cos(__q__(4)) + cos(__q__(1) + __q__(2) + __q__(3))*sin(__q__(0))*sin(__q__(4))) + d(3)*cos(__q__(0)) - a(1)*cos(__q__(1))*sin(__q__(0)) - d(4)*sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(0)) - a(2)*cos(__q__(1))*cos(__q__(2))*sin(__q__(0)) + a(2)*sin(__q__(0))*sin(__q__(1))*sin(__q__(2)),
        0,
        0,
        0,
        1
    );

    /* second column of the geometric jacobian */
    Vector6d J2(
        -cos(__q__(0))*(a(2)*sin(__q__(1) + __q__(2)) + a(1)*sin(__q__(1)) + d(4)*(sin(__q__(1) + __q__(2))*sin(__q__(3)) - cos(__q__(1) + __q__(2))*cos(__q__(3))) - d(4)*sin(__q__(4))*(cos(__q__(1) + __q__(2))*sin(__q__(3)) + sin(__q__(1) + __q__(2))*cos(__q__(3)))),
        -sin(__q__(0))*(a(2)*sin(__q__(1) + __q__(2)) + a(1)*sin(__q__(1)) + d(4)*(sin(__q__(1) + __q__(2))*sin(__q__(3)) - cos(__q__(1) + __q__(2))*cos(__q__(3))) - d(4)*sin(__q__(4))*(cos(__q__(1) + __q__(2))*sin(__q__(3)) + sin(__q__(1) + __q__(2))*cos(__q__(3)))),
        a(2)*cos(__q__(1) + __q__(2)) - (d(4)*sin(__q__(1) + __q__(2) + __q__(3) + __q__(4)))/2 + a(1)*cos(__q__(1)) + (d(4)*sin(__q__(1) + __q__(2) + __q__(3) - __q__(4)))/2 + d(4)*sin(__q__(1) + __q__(2) + __q__(3)),
        sin(__q__(0)),
        -cos(__q__(0)),
        0
    );

    /* third column of the geometric jacobian */
    Vector6d J3(
        cos(__q__(0))*(d(4)*cos(__q__(1) + __q__(2) + __q__(3)) - a(2)*sin(__q__(1) + __q__(2)) + d(4)*sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(4))),
        sin(__q__(0))*(d(4)*cos(__q__(1) + __q__(2) + __q__(3)) - a(2)*sin(__q__(1) + __q__(2)) + d(4)*sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(4))),
        a(2)*cos(__q__(1) + __q__(2)) - (d(4)*sin(__q__(1) + __q__(2) + __q__(3) + __q__(4)))/2 + (d(4)*sin(__q__(1) + __q__(2) + __q__(3) - __q__(4)))/2 + d(4)*sin(__q__(1) + __q__(2) + __q__(3)),
        sin(__q__(0)),
        -cos(__q__(0)),
        0
    );

    /* forth column of the geometric jacobian */
    Vector6d J4(
        d(4)*cos(__q__(0))*(cos(__q__(1) + __q__(2) + __q__(3)) + sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(4))),
        d(4)*sin(__q__(0))*(cos(__q__(1) + __q__(2) + __q__(3)) + sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(4))),
        d(4)*(sin(__q__(1) + __q__(2) + __q__(3) - __q__(4))/2 + sin(__q__(1) + __q__(2) + __q__(3)) - sin(__q__(1) + __q__(2) + __q__(3) + __q__(4))/2),
        sin(__q__(0)),
        -cos(__q__(0)),
        0
    );

    /* fifth column of the geometric jacobian */
    Vector6d J5(
        d(4)*cos(__q__(0))*cos(__q__(1))*cos(__q__(4))*sin(__q__(2))*sin(__q__(3)) - d(4)*cos(__q__(0))*cos(__q__(1))*cos(__q__(2))*cos(__q__(3))*cos(__q__(4)) - d(4)*sin(__q__(0))*sin(__q__(4)) + d(4)*cos(__q__(0))*cos(__q__(2))*cos(__q__(4))*sin(__q__(1))*sin(__q__(3)) + d(4)*cos(__q__(0))*cos(__q__(3))*cos(__q__(4))*sin(__q__(1))*sin(__q__(2)),
        d(4)*cos(__q__(0))*sin(__q__(4)) + d(4)*cos(__q__(1))*cos(__q__(4))*sin(__q__(0))*sin(__q__(2))*sin(__q__(3)) + d(4)*cos(__q__(2))*cos(__q__(4))*sin(__q__(0))*sin(__q__(1))*sin(__q__(3)) + d(4)*cos(__q__(3))*cos(__q__(4))*sin(__q__(0))*sin(__q__(1))*sin(__q__(2)) - d(4)*cos(__q__(1))*cos(__q__(2))*cos(__q__(3))*cos(__q__(4))*sin(__q__(0)),
        -d(4)*(sin(__q__(1) + __q__(2) + __q__(3) - __q__(4))/2 + sin(__q__(1) + __q__(2) + __q__(3) + __q__(4))/2),
        sin(__q__(1) + __q__(2) + __q__(3))*cos(__q__(0)),
        sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(0)),
        -cos(__q__(1) + __q__(2) + __q__(3))
    );

    /* sixth column of the geometric jacobian */
    Vector6d J6(
        0,
        0,
        0,
        cos(__q__(4))*sin(__q__(0)) - cos(__q__(1) + __q__(2) + __q__(3))*cos(__q__(0))*sin(__q__(4)),
        -cos(__q__(0))*cos(__q__(4)) - cos(__q__(1) + __q__(2) + __q__(3))*sin(__q__(0))*sin(__q__(4)),
        -sin(__q__(1) + __q__(2) + __q__(3))*sin(__q__(4))   
    );

    /* assign all columns to the geometric jacobian */
    __geometric_jacobian__.col(0) = J1;
    __geometric_jacobian__.col(1) = J2;
    __geometric_jacobian__.col(2) = J3;
    __geometric_jacobian__.col(3) = J4;
    __geometric_jacobian__.col(4) = J5;
    __geometric_jacobian__.col(5) = J6;

    __inverse_geometric_jacobian__ = __geometric_jacobian__.inverse();
}

Eigen::Quaterniond UR5::from_rotational_matrix_to_quaternion(Eigen::Matrix3d rotation_matrix)
{
    Eigen::Quaterniond quaternion(rotation_matrix);
	return quaternion;
}

Eigen::Vector3d UR5::compute_desired_cartesian_position(double normalized_time)
{
    Eigen::Vector3d desired_cartesian_position = normalized_time*__final_position__+(1-normalized_time)*__initial_position__;
    return desired_cartesian_position; 
}

Eigen::Quaterniond UR5::compute_desired_quaternion(double normalized_time)
{
    Eigen::Quaterniond desired_quaternion = __initial_quaternion__.slerp(normalized_time, __final_quaternion__);
    return desired_quaternion; 
}

Eigen::Vector3d UR5::calculate_angular_velocity(double normalized_time)
{
    Eigen::Quaterniond quaternion_velocity = (compute_desired_quaternion(normalized_time+__delta_time__)*compute_desired_quaternion(normalized_time).conjugate()); 
    Eigen::Vector3d angular_velocity = (quaternion_velocity.vec()*2)/__delta_time__;
    return angular_velocity; 
}

Eigen::Vector3d UR5::calculate_cartesian_position_velocity(double normalized_time) 
{
    Eigen::Vector3d velocita_end_effector_cartesiana = (compute_desired_cartesian_position(normalized_time)-compute_desired_cartesian_position(normalized_time-__delta_time__))/__delta_time__; 
    return velocita_end_effector_cartesiana; 
}

Eigen::Quaterniond UR5::compute_quaternion_error(double normalized_time)
{
    Eigen::Quaterniond quaternion_error = compute_desired_quaternion(normalized_time)*from_rotational_matrix_to_quaternion(__Re__).conjugate(); 
    return quaternion_error; 
}

Eigen::Vector3d UR5::compute_cartesian_position_error(double normalized_time)
{
    Eigen::Vector3d cartesian_position_error = compute_desired_cartesian_position(normalized_time)-__pe__; 
    return cartesian_position_error; 
}

Eigen::VectorXd UR5::joints_velocity_calculation(double normalized_time) 
{
    Eigen::Vector3d cartesian_input = calculate_cartesian_position_velocity(normalized_time)+__Kp__*compute_cartesian_position_error(normalized_time); 
    Eigen::Vector3d quaternion_input = calculate_angular_velocity(normalized_time)+__Kq__*(compute_quaternion_error(normalized_time)).vec(); 
    Eigen::VectorXd jacobin_input(6); 
    jacobin_input << cartesian_input, quaternion_input;

    Eigen::VectorXd joints_velocity(6); 
    joints_velocity = __inverse_geometric_jacobian__*jacobin_input;
    return joints_velocity; 
}

/* compute the next joint position */
Eigen::VectorXd UR5::calculate_next_joints(double normalized_time)
{
    Eigen::VectorXd next_joints = __q__+joints_velocity_calculation(normalized_time)*__delta_time__; 
    return next_joints; 
}

double UR5::time_normalization(double time, double max_time) 
{
    double normalizedTime = time/max_time;
    return normalizedTime;
}

/* motion function */
void UR5::motion_plan(Eigen::Vector3d final_point, Eigen::Matrix3d final_rotation_matrix)
{   
    /* Adjust the publishing rate as needed (e.g., 10 Hz) */
    ros::Rate rate(10); 
    std_msgs::Float64MultiArray jointCommand;

    /* define Kp and Kq */
    __Kq__ = Eigen::Matrix3d::Identity()*10; 
    __Kp__ = Eigen::Matrix3d::Identity()*10; 

    /* initialize the initial and final quaternian of the trajectory */
    __initial_quaternion__ = from_rotational_matrix_to_quaternion(__Re__); 
    __final_quaternion__ = from_rotational_matrix_to_quaternion(final_rotation_matrix); 

    /* initialize the initial and final position of the trajectory */
    __initial_position__ = __pe__; 
    __final_position__ = final_point; 

    /* max time of the motion */
    const double max_time = 100;

    /* normalization time init */
    double t = 0;

    /* start the movemt */
    while (ros::ok()) 
    {
        std::cout << "t = " << t << std::endl;
        /* next joints to approach */
        Eigen::VectorXd next_joints;
        if (t <= max_time)
        {
            double t_normalizzato = time_normalization(t, max_time);
            next_joints = calculate_next_joints(t_normalizzato);
            jointCommand.data = {next_joints(0), next_joints(1), next_joints(2), next_joints(3), next_joints(4), next_joints(5), 0.0, 0.0};
            t += __delta_time__;
        }
        else 
        {
            jointCommand.data = {__q__(0), __q__(1), __q__(2), __q__(3), __q__(4), __q__(5), 0.0, 0.0};
        }

        pub.publish(jointCommand);
        ros::spinOnce();
        rate.sleep();
    }
}
