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
    __q__ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

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
    nh.subscribe("/ur5/joint_states", 10, &UR5::update_q, this);

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
}

/* given arotation matrix this function returns its respective unique quaternion */
Eigen::Quaterniond UR5::from_rotational_matrix_to_quaternion(Eigen::Matrix3d rotation_matrix)
{
    Eigen::Quaterniond quaternion(rotation_matrix);
	return quaternion;
}

/* motion function */
void UR5::motion_plan(Eigen::Vector3d final_point, Eigen::Matrix3d final_rotation_matrix)
{   
    /* Adjust the publishing rate as needed (e.g., 10 Hz) */
    ros::Rate rate(10); 
    std_msgs::Float64MultiArray jointCommand;

    /* define Kp and Kq */
    Eigen::Matrix3d Kq = Eigen::Matrix3d::Identity()*10; 
    std::cout << "Kq = " << std::endl << Kq << std::endl;
    Eigen::Matrix3d Kp = Eigen::Matrix3d::Identity()*10; 
    std::cout << "Kp = " << std::endl << Kq << std::endl;

    /* initialize the initial and final quaternian of the trajectory */
    Eigen::Quaterniond initial_quaternion = from_rotational_matrix_to_quaternion(__Re__); 
    std::cout << "initial_quaternion = " << std::endl << initial_quaternion << std::endl;
    Eigen::Quaterniond final_quaternion = from_rotational_matrix_to_quaternion(final_rotation_matrix); 
    std::cout << "final_quaternion = " << std::endl << final_quaternion << std::endl;

    /* initialize the initial and final position of the trajectory */
    Eigen::Vector3d initial_position = __pe__; 
    std::cout << "initial_position = " << std::endl << initial_position << std::endl;
    Eigen::Vector3d final_position = final_point; 
    std::cout << "final_position = " << std::endl << final_position << std::endl;

    /* initialize the computetion frame for each step */
    const double delta_time = 0.05;

    /* max time of the motion */
    const double max_time = 100;

    /* computer the inverse geometric jacobain */
    Eigen::Matrix<double, 6, 6> inverse_geometric_jacobian = __geometric_jacobian__.inverse(); 

    /* compute the next desired position of the end_effector in the trajectory */
    auto get_desired_cartesion_position = [&] (double time)
    {
        /* x(t) = x_i*t + (1 - t)*x_f */
        Eigen::Vector3d position_in_time_time = time*final_position+(1-time)*initial_position;
        return position_in_time_time; 
    };

    /* compute the next desired quaternion of the trajectory according to slerp function */
    auto get_desired_quaternion = [&] (double time) 
    {
        /* slerp function computation */
        Eigen::Quaterniond desired_quaternion = initial_quaternion.slerp(time, final_quaternion);
        return desired_quaternion; 
    };

    /* compute the angular velocity according to the trajectory to follow */
    auto calculate_angular_velocity = [&] (double time) 
    {
        Eigen::Quaterniond quaternion_velocity = (get_desired_quaternion(time+delta_time)*get_desired_quaternion(time).conjugate()); 
        Eigen::Vector3d angular_velocity = (quaternion_velocity.vec()*2)/delta_time;
        return angular_velocity; 
    };

    /* compute the velocity according to the trajectory to follow */
    auto cartesian_velocity_desired = [&] (double time) 
    {
        Eigen::Vector3d velocita_end_effector_cartesiana = (get_desired_cartesion_position(time)-get_desired_cartesion_position(time-delta_time))/delta_time; 
        return velocita_end_effector_cartesiana; 
    };

    /* compute the rotational error expressed in quaternion */
    auto compute_quaternion_error = [&] (double time) 
    {
        Eigen::Quaterniond quaternion_error = get_desired_quaternion(time)*from_rotational_matrix_to_quaternion(__Re__).conjugate(); 
        return quaternion_error; 
    };

    /* compute the cartesion position error respect to the trajectory */
    auto compute_cartesian_position_error = [&] (double time) 
    {
        Eigen::Vector3d cartesian_position_error = get_desired_cartesion_position(time)-__pe__; 
        return cartesian_position_error; 
    };

    /* compute the joints velocities  */
    auto joints_velocity_calculation = [&] (double time) 
    {
        Eigen::Vector3d cartesian_input = cartesian_velocity_desired(time)+Kp*compute_cartesian_position_error(time); 
        Eigen::Vector3d quaternion_input = calculate_angular_velocity(time)+Kq*(compute_quaternion_error(time)).vec(); 
        Eigen::VectorXd jacobin_input(6); 
        jacobin_input << cartesian_input, quaternion_input;

        Eigen::VectorXd joints_velocity(6); 
        joints_velocity = inverse_geometric_jacobian*jacobin_input;
        return joints_velocity; 
    };

    /* compute the next joint position */
    auto calculate_next_joints = [&] (double tempo)
    {
        Eigen::VectorXd next_joints = __q__+joints_velocity_calculation(tempo)*delta_time; 
        return next_joints; 
    };

    /* normalization of the time respect to the max_time */
    auto time_normalization = [&] (double time) 
    {
        double normalizedTime = time / max_time;
        return normalizedTime;
    };

    /* start the movemt */
    while (ros::ok()) 
    {
        /* normalization time init */
        double t = 0;

        /* next joints to approach */
        Eigen::VectorXd next_joints;
        if (t <= max_time)
        {
            double t_normalizzato = time_normalization(t);
            next_joints = calculate_next_joints(t_normalizzato);
            jointCommand.data = {next_joints(0), next_joints(1), next_joints(2), next_joints(3), next_joints(4), next_joints(5), 0.0, 0.0};
            t += delta_time;
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
