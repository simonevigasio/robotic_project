// Header
#include "robotic_project/kinematics.h"

// ROS
#include "sensor_msgs/JointState.h"
#include "boost/shared_ptr.hpp"
#include "ros/ros.h"

// Eigen 
#include "Eigen/Dense"
#include "Eigen/QR"

// Standard Eige
#include <iostream>
#include <cmath>

// Read the q values from the UR5 
Vector6d read_q()
{
    // Read the eight q values => six for the arm and two for the gripper
    boost::shared_ptr<sensor_msgs::JointState const> msg;
    msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");

    // Controll if the message is received correctly
    if (msg == NULL) throw; 

    // Store the information and arrange it properly
    Eigen::VectorXd q(8); for (int i=0; i<8; ++i) q(i) = msg->position[i];
    Vector6d arm(q(4), q(3), q(0), q(5), q(6), q(7));
    return arm;
}

// Convertion from euler angles to a rotation matrix
Eigen::Matrix3d from_euler_angles_to_rotation_matrix(Eigen::Vector3d euler_angles)
{
    const double phi = euler_angles(0);
    const double theta = euler_angles(1);
    const double gamma = euler_angles(2);

    return Eigen::Matrix3d {
            {cos(phi)*cos(theta), cos(phi)*sin(theta)*sin(gamma)-sin(phi)*cos(gamma), cos(phi)*sin(theta)*cos(gamma)+sin(phi)*sin(gamma)},
            {sin(phi)*cos(theta), sin(phi)*sin(theta)*sin(gamma)+cos(phi)*cos(gamma), sin(phi)*sin(theta)*cos(gamma)-cos(phi)*sin(gamma)},
            {-sin(theta), cos(theta)*sin(gamma), cos(theta)*cos(gamma)}
    };
}

// Convertion from rotation matrix to euler angles 
Eigen::Vector3d from_rotation_matrix_to_euler_angles(Eigen::Matrix3d rotation_matrix)
{
    double phi = atan2(rotation_matrix(1,0),rotation_matrix(0,0));
    double theta = atan2(-rotation_matrix(2,0), sqrt(pow(rotation_matrix(2,1), 2) + pow(rotation_matrix(2,2), 2)));
    double gamma = atan2(rotation_matrix(2,1), rotation_matrix(2,2));
    Eigen::Vector3d euler_angles(phi, theta, gamma);
    return euler_angles;
}

// Generate T from i-1 to i given the DH parameters 
Eigen::Matrix4d generate_transformation_matrix(double a, double alpha, double d, double theta)
{
    return Eigen::Matrix4d {
        {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)},
        {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)},
        {0, sin(alpha), cos(alpha), d},
        {0, 0, 0, 1}
    };
}

// Direct Kinematics 2
Eigen::Matrix4d direct_kinematics_2(Vector6d q)
{
    Eigen::Matrix4d T10f {
            {cos(q(0)) , -sin(q(0)) , 0 , 0},
            {sin(q(0)) , cos(q(0)) , 0 , 0},
            {0 , 0 , 1 , d(0)},
            {0 , 0 , 0 , 1}
    };

    Eigen::Matrix4d T21f {
            {cos(q(1)) , -sin(q(1)) , 0 , 0},
            {0 , 0 , -1 , 0},
            {sin(q(1)) , cos(q(1)) , 0 , 0},
            {0 , 0 , 0 , 1}
    };

    Eigen::Matrix4d T32f {
            {cos(q(2)) , -sin(q(2)) , 0 , a(1)},
            {sin(q(2)) , cos(q(2)) , 0 , 0},
            {0 , 0 , 1 , d(2)},
            {0 , 0 , 0 , 1}
    };

    Eigen::Matrix4d T43f {
            {cos(q(3)) , -sin(q(3)) , 0 , a(2)},
            {sin(q(3)) , cos(q(3)) , 0 , 0},
            {0 , 0 , 1 , d(3)},
            {0 , 0 , 0 , 1}
    };

    Eigen::Matrix4d T54f {
            {cos(q(4)) , -sin(q(4)) , 0 , 0},
            {0 , 0 , -1 , -d(4)},
            {sin(q(4)) , cos(q(4)) , 0 , 0},
            {0 , 0 , 0 , 1}
    };

    Eigen::Matrix4d T65f {
            {cos(q(5)) , -sin(q(5)) , 0 , 0},
            {0 , 0 , 1 , d(5)},
            {-sin(q(5)) , -cos(q(5)) , 0 , 0},
            {0 , 0 , 0 , 1}
    };

    return T10f*T21f*T32f*T43f*T54f*T65f;
}

// Direct kinematics
Eigen::Matrix4d direct_kinematics(Vector6d q)
{
    Eigen::Matrix4d T10 = generate_transformation_matrix(a(0), alpha(0), d(0), q(0));
    Eigen::Matrix4d T21 = generate_transformation_matrix(a(1), alpha(1), d(1), q(1));
    Eigen::Matrix4d T32 = generate_transformation_matrix(a(2), alpha(2), d(2), q(2));
    Eigen::Matrix4d T43 = generate_transformation_matrix(a(3), alpha(3), d(3), q(3));
    Eigen::Matrix4d T54 = generate_transformation_matrix(a(4), alpha(4), d(4), q(4));
    Eigen::Matrix4d T65 = generate_transformation_matrix(a(5), alpha(5), d(5), q(5));
    Eigen::Matrix4d T60 = T10*T21*T32*T43*T54*T65;
    return T60;
}

// Inverse kinematics
Eigen::Matrix<double, 6, 8> inverse_kinematics(Eigen::Vector3d P60, Eigen::Matrix3d R60)
{
    Eigen::Matrix4d T60 {
        {R60(0,0), R60(0,1), R60(0,2), P60(0)}, 
        {R60(1,0), R60(1,1), R60(1,2), P60(1)},
        {R60(2,0), R60(2,1), R60(2,2), P60(2)},
        {0, 0, 0, 1}
    };

    Eigen::Vector4d V {0, 0, -d(5), 1};
    Eigen::Vector4d P50 = T60*V;

    double psi = atan2(P50(1), P50(0));
    double distance_P50x_P50y = hypot(P50(1), P50(0));

    if (distance_P50x_P50y < d(3))
    {
        ROS_INFO("Position request in the unreachable cylinder");
        throw;
    }

    double phi1 = acos(d(3)/distance_P50x_P50y);
    double phi2 = -phi1;

    // θ1
    double theta1_1 = psi+phi1+M_PI/2;
    double theta1_2 = psi+phi2+M_PI/2;

    double p61z_1 = P60(0)*sin(theta1_1)-P60(1)*cos(theta1_1);
    double p61z_2 = P60(0)*sin(theta1_2)-P60(1)*cos(theta1_2);

    // θ5
    double theta5_1_1 = acos((p61z_1-d(3))/d(5));
    double theta5_1_2 = -acos((p61z_1-d(3))/d(5));
    double theta5_2_1 = acos((p61z_2-d(3))/d(5));
    double theta5_2_2 = -acos((p61z_2-d(3))/d(5));

    Eigen::Matrix4d T10_1 = generate_transformation_matrix(a(0), alpha(0), d(0), theta1_1);
    Eigen::Matrix4d T10_2 = generate_transformation_matrix(a(0), alpha(0), d(0), theta1_2);

    Eigen::Matrix4d T16_1 = ((T10_1.inverse())*T60).inverse();
    Eigen::Matrix4d T16_2 = ((T10_2.inverse())*T60).inverse();

    double Zy_1 = T16_1(1,2);
    double Zy_2 = T16_2(1,2);
    double Zx_1 = T16_1(0,2);
    double Zx_2 = T16_2(0,2);

    // θ6
    double theta6_1_1 = 0;
    double theta6_2_1 = 0;
    double theta6_1_2 = 0;
    double theta6_2_2 = 0;

    if (is_zero(sin(theta5_1_1)) || (is_zero(Zy_1) && is_zero(Zx_1))) ROS_INFO("Singular configuration. Choosing arbitrary theta6");
    else theta6_1_1 = atan2(((-Zy_1)/sin(theta5_1_1)),(Zx_1/sin(theta5_1_1)));

    if (is_zero(sin(theta5_1_2)) || (is_zero(Zy_1) && is_zero(Zx_1))) ROS_INFO("Singular configuration. Choosing arbitrary theta6");
    else theta6_1_2 = atan2(((-Zy_1)/sin(theta5_1_2)),(Zx_1/sin(theta5_1_2)));

    if (is_zero(theta5_2_1) || (is_zero(Zy_2) && is_zero(Zx_2))) ROS_INFO("Singular configuration. Choosing arbitrary theta6");
    else theta6_2_1 = atan2(((-Zy_2)/sin(theta5_2_1)),(Zx_2/sin(theta5_2_1)));

    if (is_zero(theta5_2_2) || (is_zero(Zy_2) && is_zero(Zx_2))) ROS_INFO("Singular configuration. Choosing arbitrary theta6");
    else theta6_2_2 = atan2(((-Zy_2)/sin(theta5_2_2)),(Zx_2/sin(theta5_2_2)));

     // θ3
    Eigen::Matrix4d T61_1 = T16_1.inverse();
    Eigen::Matrix4d T61_2 = T16_2.inverse();

    Eigen::Matrix4d T54_1_1 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_1_1);
    Eigen::Matrix4d T54_1_2 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_1_2);
    Eigen::Matrix4d T54_2_1 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_2_1);
    Eigen::Matrix4d T54_2_2 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_2_2);

    Eigen::Matrix4d T65_1_1 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_1_1);
    Eigen::Matrix4d T65_1_2 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_1_2);
    Eigen::Matrix4d T65_2_1 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_2_1);
    Eigen::Matrix4d T65_2_2 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_2_2);

    Eigen::Matrix4d T41_1_1 = T61_1*((T54_1_1*T65_1_1).inverse());
    Eigen::Matrix4d T41_1_2 = T61_1*((T54_1_2*T65_1_2).inverse());
    Eigen::Matrix4d T41_2_1 = T61_2*((T54_2_1*T65_2_1).inverse());
    Eigen::Matrix4d T41_2_2 = T61_2*((T54_2_2*T65_2_2).inverse());

    V << 0.0, -d(3), 0.0, 1.0; Eigen::Vector4d P;
    P = T41_1_1*V; Eigen::Vector3d P31_1_1(P(0), P(1), P(2));
    P = T41_1_2*V; Eigen::Vector3d P31_1_2(P(0), P(1), P(2));
    P = T41_2_1*V; Eigen::Vector3d P31_2_1(P(0), P(1), P(2));
    P = T41_2_2*V; Eigen::Vector3d P31_2_2(P(0), P(1), P(2));

    double C = (P31_1_1.squaredNorm()-pow(a(1),2.0)-pow(a(2),2.0))/(2*a(1)*a(2));
    double theta3_1_1_1;
    double theta3_1_1_2;

    if (abs(C) > 1) ROS_INFO("Point out of the work space");
    else 
    {
        theta3_1_1_1 = acos(C);
        theta3_1_1_2 = -acos(C);
    }

    C = (P31_1_2.squaredNorm()-pow(a(1),2.0)-pow(a(2),2.0))/(2*a(1)*a(2));
    double theta3_1_2_1;
    double theta3_1_2_2;

    if (abs(C) > 1) ROS_INFO("Point out of the work space");
    else
    {
        theta3_1_2_1 = acos(C);
        theta3_1_2_2 = -acos(C);
    }

    C = (P31_2_1.squaredNorm()-pow(a(1),2.0)-pow(a(2),2.0))/(2*a(1)*a(2));
    double theta3_2_1_1;
    double theta3_2_1_2;

    if (abs(C) > 1) ROS_INFO("Point out of the work space");
    else 
    {
        theta3_2_1_1 = acos(C);
        theta3_2_1_2 = -acos(C);
    }

    C = (P31_2_2.squaredNorm()-pow(a(1),2.0)-pow(a(2),2.0))/(2*a(1)*a(2));
    double theta3_2_2_1;
    double theta3_2_2_2;

    if (abs(C) > 1) ROS_INFO("Point out of the work space");
    else 
    {
        theta3_2_2_1 = acos(C);
        theta3_2_2_2 = -acos(C);
    }

    // θ2
    double theta2_1_1_1 = -atan2(P31_1_1(1), -P31_1_1(0))+asin((a(2)*sin(theta3_1_1_2))/P31_1_1.norm());
    double theta2_1_1_2 = -atan2(P31_1_1(2), -P31_1_1(1))+asin((a(2)*sin(theta3_1_1_2))/P31_1_1.norm());
    double theta2_1_2_1 = -atan2(P31_1_2(2), -P31_1_2(1))+asin((a(2)*sin(theta3_1_2_1))/P31_1_2.norm());
    double theta2_1_2_2 = -atan2(P31_1_2(2), -P31_1_2(1))+asin((a(2)*sin(theta3_1_2_2))/P31_1_2.norm());
    double theta2_2_1_1 = -atan2(P31_2_1(2), -P31_2_1(1))+asin((a(2)*sin(theta3_2_1_1))/P31_2_1.norm());
    double theta2_2_1_2 = -atan2(P31_2_1(2), -P31_2_1(1))+asin((a(2)*sin(theta3_2_1_2))/P31_2_1.norm());
    double theta2_2_2_1 = -atan2(P31_2_2(2), -P31_2_2(1))+asin((a(2)*sin(theta3_2_2_1))/P31_2_2.norm());
    double theta2_2_2_2 = -atan2(P31_2_2(2), -P31_2_2(1))+asin((a(2)*sin(theta3_2_2_2))/P31_2_2.norm());

    // θ4 
    Eigen::Matrix4d T21_1_1_1 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_1_1_1);
    Eigen::Matrix4d T32_1_1_1 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_1_1_1);
    Eigen::Matrix4d T43_1_1_1 = (T21_1_1_1*T32_1_1_1).inverse()*T41_1_1;
    double xy = T43_1_1_1(1,0);
    double xx = T43_1_1_1(0,0);
    double theta4_1_1_1 = atan2(xy, xx);

    Eigen::Matrix4d T21_1_1_2 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_1_1_2);
    Eigen::Matrix4d T32_1_1_2 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_1_1_2);
    Eigen::Matrix4d T43_1_1_2 = (T21_1_1_2*T32_1_1_2).inverse()*T41_1_2;
    xy = T43_1_1_2(1,0);
    xx = T43_1_1_2(0,0);
    double theta4_1_1_2 = atan2(xy, xx);

    Eigen::Matrix4d T21_1_2_1 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_1_2_1);
    Eigen::Matrix4d T32_1_2_1 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_1_2_1);
    Eigen::Matrix4d T43_1_2_1 = (T21_1_2_1*T32_1_2_1).inverse()*T41_2_1;
    xy = T43_1_2_1(1,0);
    xx = T43_1_2_1(0,0);
    double theta4_1_2_1 = atan2(xy, xx);

    Eigen::Matrix4d T21_1_2_2 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_1_2_2);
    Eigen::Matrix4d T32_1_2_2 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_1_2_2);
    Eigen::Matrix4d T43_1_2_2 = (T21_1_2_2*T32_1_2_2).inverse()*T41_2_2;
    xy = T43_1_2_2(1,0);
    xx = T43_1_2_2(0,0);
    double theta4_1_2_2 = atan2(xy, xx);

    Eigen::Matrix4d T21_2_1_1 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_2_1_1);
    Eigen::Matrix4d T32_2_1_1 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_2_1_1);
    Eigen::Matrix4d T43_2_1_1 = (T21_2_1_1*T32_2_1_1).inverse()*T41_1_1;
    xy = T43_2_1_1(1,0);
    xx = T43_2_1_1(0,0);
    double theta4_2_1_1 = atan2(xy, xx);

    Eigen::Matrix4d T21_2_1_2 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_2_1_2);
    Eigen::Matrix4d T32_2_1_2 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_2_1_2);
    Eigen::Matrix4d T43_2_1_2 = (T21_2_1_2*T32_2_1_2).inverse()*T41_1_2;
    xy = T43_2_1_2(1,0);
    xx = T43_2_1_2(0,0);
    double theta4_2_1_2 = atan2(xy, xx);

    Eigen::Matrix4d T21_2_2_1 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_2_2_1);
    Eigen::Matrix4d T32_2_2_1 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_2_2_1);
    Eigen::Matrix4d T43_2_2_1 = (T21_2_2_1*T32_2_2_1).inverse()*T41_2_1;
    xy = T43_2_2_1(1,0);
    xx = T43_2_2_1(0,0);
    double theta4_2_2_1 = atan2(xy, xx);

    Eigen::Matrix4d T21_2_2_2 = generate_transformation_matrix(a(1), alpha(1), d(1), theta2_2_2_2);
    Eigen::Matrix4d T32_2_2_2 = generate_transformation_matrix(a(2), alpha(2), d(2), theta3_2_2_2);
    Eigen::Matrix4d T43_2_2_2 = (T21_2_2_2*T32_2_2_2).inverse()*T41_2_2;
    xy = T43_2_2_2(1,0);
    xx = T43_2_2_2(0,0);
    double theta4_2_2_2 = atan2(xy, xx);

    // Configurations
    Eigen::Matrix<double, 6, 8> Confs {
        {theta1_1, theta1_1, theta1_1, theta1_1, theta1_2, theta1_2, theta1_2, theta1_2},
        {theta2_1_1_1, theta2_1_1_2, theta2_1_2_1, theta2_1_2_2, theta2_2_1_1, theta2_2_1_2, theta2_2_2_1, theta2_2_2_2},
        {theta3_1_1_1, theta3_1_1_2, theta3_1_2_1, theta3_1_2_2, theta3_2_1_1, theta3_2_1_2, theta3_2_2_1, theta3_2_2_2},
        {theta4_1_1_1, theta4_1_1_2, theta4_1_2_1, theta4_1_2_2, theta4_2_1_1, theta4_2_1_2, theta4_2_2_1, theta4_2_2_2},
        {theta5_1_1, theta5_1_1, theta5_1_2, theta5_1_2, theta5_2_1, theta5_2_1, theta5_2_2, theta5_2_2},
        {theta6_1_1, theta6_1_1, theta6_1_2, theta6_1_2, theta6_2_1, theta6_2_1, theta6_2_2, theta6_2_2}
    };

    return Confs;
}

// Control if a double is zero
bool is_zero(double x)
{
    if (abs(x) < 0.0000001) return true;
    return false;
}

// Geometric jacobian
Eigen::Matrix<double, 6, 6> generate_geometric_jacobian(Vector6d q)
{
    Eigen::Vector3d P00(0.0, 0.0, 0.0);
    Eigen::Vector3d z00(0.0, 0.0, 1.0);

    Eigen::Matrix4d T10 = generate_transformation_matrix(a(0), alpha(0), d(0), q(0));
    Eigen::Matrix3d R10 = T10.block<3,3>(0,0);
    Eigen::Vector3d P10 = T10.block<3,1>(0,3);
    Eigen::Vector3d z10 = R10*z00;

    Eigen::Matrix4d T21 = generate_transformation_matrix(a(1), alpha(1), d(1), q(1));
    Eigen::Matrix4d T20 = T10*T21;
    Eigen::Matrix3d R20 = T20.block<3,3>(0,0);
    Eigen::Vector3d P20 = T20.block<3,1>(0,3);
    Eigen::Vector3d z20 = R20*z00;

    Eigen::Matrix4d T32 = generate_transformation_matrix(a(2), alpha(2), d(2), q(2));
    Eigen::Matrix4d T30 = T20*T32;
    Eigen::Matrix3d R30 = T30.block<3,3>(0,0);
    Eigen::Vector3d P30 = T30.block<3,1>(0,3);
    Eigen::Vector3d z30 = R30*z00;

    Eigen::Matrix4d T43 = generate_transformation_matrix(a(3), alpha(3), d(3), q(3));
    Eigen::Matrix4d T40 = T30*T43;
    Eigen::Matrix3d R40 = T40.block<3,3>(0,0);
    Eigen::Vector3d P40 = T40.block<3,1>(0,3);
    Eigen::Vector3d z40 = R40*z00;

    Eigen::Matrix4d T54 = generate_transformation_matrix(a(4), alpha(4), d(4), q(4));
    Eigen::Matrix4d T50 = T40*T54;
    Eigen::Matrix3d R50 = T50.block<3,3>(0,0);
    Eigen::Vector3d P50 = T50.block<3,1>(0,3);
    Eigen::Vector3d z50 = R50*z00;

    Eigen::Matrix4d T65 = generate_transformation_matrix(a(5), alpha(5), d(5), q(5));
    Eigen::Matrix4d T60 = T50*T65;
    Eigen::Vector3d P60 = T60.block<3,1>(0,3);

    Vector6d J0; J0 << z00.cross(P60-P00), z00;
    Vector6d J1; J1 << z10.cross(P60-P10), z10;
    Vector6d J2; J2 << z20.cross(P60-P20), z20;
    Vector6d J3; J3 << z30.cross(P60-P30), z30;
    Vector6d J4; J4 << z40.cross(P60-P40), z40;
    Vector6d J5; J5 << z50.cross(P60-P50), z50;

    Eigen::Matrix<double, 6, 6> geometric_jacobain;
    geometric_jacobain.col(0) = J0;
    geometric_jacobain.col(1) = J1;
    geometric_jacobain.col(2) = J2;
    geometric_jacobain.col(3) = J3;
    geometric_jacobain.col(4) = J4;
    geometric_jacobain.col(5) = J5;
    return geometric_jacobain;
}

// Geometric Jacobian
Eigen::Matrix<double, 6, 6> generate_ur5_geometric_jacobian(Vector6d q)
{
    Vector6d J0(
        d(4)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + d(3)*cos(q(0)) - a(1)*cos(q(1))*sin(q(0)) - d(4)*sin(q(1) + q(2) + q(3))*sin(q(0)) - a(2)*cos(q(1))*cos(q(2))*sin(q(0)) + a(2)*sin(q(0))*sin(q(1))*sin(q(2)),
        d(4)*(cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4))) + d(3)*sin(q(0)) + a(1)*cos(q(0))*cos(q(1)) + d(4)*sin(q(1) + q(2) + q(3))*cos(q(0)) + a(2)*cos(q(0))*cos(q(1))*cos(q(2)) - a(2)*cos(q(0))*sin(q(1))*sin(q(2)),
        0,
        0,
        0,
        1
    );

    Vector6d J1(
        -cos(q(0))*(a(2)*sin(q(1) + q(2)) + a(1)*sin(q(1)) + d(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - d(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
        -sin(q(0))*(a(2)*sin(q(1) + q(2)) + a(1)*sin(q(1)) + d(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - d(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
        a(2)*cos(q(1) + q(2)) - (d(4)*sin(q(1) + q(2) + q(3) + q(4)))/2 + a(1)*cos(q(1)) + (d(4)*sin(q(1) + q(2) + q(3) - q(4)))/2 + d(4)*sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0
    );

    Vector6d J2(
        -cos(q(0))*(a(2)*sin(q(1) + q(2)) + a(1)*sin(q(1)) + d(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - d(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
        -sin(q(0))*(a(2)*sin(q(1) + q(2)) + a(1)*sin(q(1)) + d(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - d(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
        a(2)*cos(q(1) + q(2)) - (d(4)*sin(q(1) + q(2) + q(3) + q(4)))/2 + a(1)*cos(q(1)) + (d(4)*sin(q(1) + q(2) + q(3) - q(4)))/2 + d(4)*sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0
    );

    Vector6d J3(
        d(4)*cos(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
        d(4)*sin(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
        d(4)*(sin(q(1) + q(2) + q(3) - q(4))/2 + sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4))/2),
        sin(q(0)),
        -cos(q(0)),
        0
    );

    Vector6d J4(
        d(4)*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2))*sin(q(3)) - d(4)*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) - d(4)*sin(q(0))*sin(q(4)) + d(4)*cos(q(0))*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + d(4)*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)),
        d(4)*cos(q(0))*sin(q(4)) + d(4)*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(3)) + d(4)*cos(q(2))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3)) + d(4)*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(2)) - d(4)*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0)),
        -d(4)*(sin(q(1) + q(2) + q(3) - q(4))/2 + sin(q(1) + q(2) + q(3) + q(4))/2),
        sin(q(1) + q(2) + q(3))*cos(q(0)),
        sin(q(1) + q(2) + q(3))*sin(q(0)),
        -cos(q(1) + q(2) + q(3))
    );

    Vector6d J5(
        0,
        0,
        0,
        cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)),
        -cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)),
        -sin(q(1) + q(2) + q(3))*sin(q(4))   
    );

    Eigen::Matrix<double, 6, 6> geometric_jacobain;
    geometric_jacobain.col(0) = J0;
    geometric_jacobain.col(1) = J1;
    geometric_jacobain.col(2) = J2;
    geometric_jacobain.col(3) = J3;
    geometric_jacobain.col(4) = J4;
    geometric_jacobain.col(5) = J5;
    return geometric_jacobain;
}

Eigen::Matrix<double, 6, 6> generate_analitycal_jacobian(Eigen::Matrix<double, 6, 6> geometric_jacobian, Eigen::Vector3d euler_angles)
{
    // Extract the angles
    double phi = euler_angles(0);
    double theta = euler_angles(1);
    double gamma = euler_angles(2);

    // Matrix used to convert angular velocity to euler angles 
    Eigen::Matrix3d T {
        {cos(theta)*cos(gamma), -sin(gamma), 0.0}, 
        {cos(theta)*sin(gamma), cos(gamma), 0.0}, 
        {-sin(theta), 0.0, 1.0}
    };

    // Control for safety of the position 
    if (abs(T.determinant()) < 0.0000001) 
    {
        ROS_INFO("Near singular configuration");
        ROS_INFO("phi: %f, theta: %f, gamma: %f", phi, theta, gamma);
    }

    // Implement the product with the geometric jacobian
    Eigen::Matrix<double, 6, 6> Ta;
    Ta << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), T;

    return Ta.inverse()*geometric_jacobian;
}

Eigen::Vector3d position_function(double t, Eigen::Vector3d initial_position, Eigen::Vector3d final_position)
{
    double nomalized_t = t/duration_trajectory;
    if (nomalized_t > 1) return final_position;
    else return nomalized_t*final_position+(1-nomalized_t)*initial_position;
}

Eigen::Quaterniond quaternion_function(double t, Eigen::Quaterniond initial_quaternian, Eigen::Quaterniond final_quaternian)
{
    double nomalized_t = t/duration_trajectory;
    if (nomalized_t > 1) return final_quaternian;
    else return initial_quaternian.slerp(nomalized_t, final_quaternian);
}

Eigen::Vector3d euler_angles_function(double t, Eigen::Vector3d initial_euler_angles, Eigen::Vector3d final_euler_angles)
{
    double normalized_t = t/duration_trajectory;
    if (normalized_t > 1) return final_euler_angles;
    else return normalized_t*final_euler_angles+(1-normalized_t)*initial_euler_angles;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> inverse_differential_kinematics(Vector6d q, Eigen::Vector3d initial_position, Eigen::Vector3d initial_euler_angles, Eigen::Vector3d final_position, Eigen::Vector3d final_euler_angles)
{
    Vector6d q_step, input_jacobian, q_derivative;
    Eigen::Matrix<double, 6, Eigen::Dynamic> trajectory = q;
    Eigen::Matrix4d transformation_matrix_step;
    Eigen::Vector3d position_step, position_velocity, euler_angles_velocity, euler_angles_step;
    Eigen::Matrix3d rotation_matrix_step;
    Eigen::Matrix<double, 6, 6> geometric_jacobian, analitycal_jacobian;

    q_step = q;

    for (double t=delta_time; t<duration_trajectory; t+=delta_time)
    {
        // Current position and euler angles
        transformation_matrix_step = direct_kinematics(q_step);
        position_step = transformation_matrix_step.block<3,1>(0,3);
        rotation_matrix_step = transformation_matrix_step.block<3,3>(0,0);
        euler_angles_step = from_rotation_matrix_to_euler_angles(rotation_matrix_step);

        // Desired velocities
        position_velocity = (position_function(t, initial_position, final_position)-position_function(t-delta_time, initial_position, final_position))/delta_time;
        euler_angles_velocity = (euler_angles_function(t, initial_euler_angles, final_euler_angles)-euler_angles_function(t-delta_time, initial_euler_angles, final_euler_angles))/delta_time;

        // Analitycal jacobian
        geometric_jacobian = generate_geometric_jacobian(q_step);
        analitycal_jacobian = generate_analitycal_jacobian(geometric_jacobian, euler_angles_step);

        // Composition of the jacobian input 
        input_jacobian << 
            position_velocity+Kp*(position_function(t, initial_position, final_position)-position_step), 
            euler_angles_velocity+Kphi*(euler_angles_function(t, initial_euler_angles, final_euler_angles)-euler_angles_step);

        // Velocity
        q_derivative = analitycal_jacobian.completeOrthogonalDecomposition().pseudoInverse()*input_jacobian;

        // Final computation
        q_step = q_step+q_derivative*delta_time;
        trajectory.conservativeResize(6, trajectory.cols()+1);
        trajectory.col(trajectory.cols()-1) = q_step;
    }

    return trajectory;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> inverse_differential_kinematics_with_quaternions(Vector6d q, Eigen::Vector3d initial_point, Eigen::Quaterniond initial_quaternion, Eigen::Vector3d final_point, Eigen::Quaterniond final_quaternion)
{
    Vector6d q_step,input_jacobian, q_derivative;
    Eigen::Matrix<double, 6, Eigen::Dynamic> trajectory = q;
    Eigen::Matrix4d transformation_matrix_step;
    Eigen::Vector3d position_step, angular_velocity, position_velocity; 
    Eigen::Matrix3d rotation_matrix_step;
    Eigen::Quaterniond quaternion_step, quaternion_error, quaternion_velocity;
    Eigen::Matrix<double, 6, 6> geometric_jacobian, inverse_geometric_jacobian;

    for (double t=delta_time; t<duration_trajectory; t+=delta_time) 
    {
        // Current position and quaternion
        transformation_matrix_step = direct_kinematics(q_step);
        position_step = transformation_matrix_step.block<3,1>(0,3);
        rotation_matrix_step = transformation_matrix_step.block<3,3>(0,0);
        quaternion_step = rotation_matrix_step;

        // Desired velocities 
        position_velocity = (position_function(t, initial_point, final_point)-position_function(t-delta_time, initial_point, final_point))/delta_time;
        quaternion_velocity = (quaternion_function(t+delta_time, initial_quaternion, final_quaternion)*quaternion_function(t, initial_quaternion, final_quaternion).conjugate()); 
	    angular_velocity = (quaternion_velocity.vec()*2)/delta_time;

        // Jacobian computation
        geometric_jacobian = generate_geometric_jacobian(q_step);
        inverse_geometric_jacobian = ((geometric_jacobian.transpose()*geometric_jacobian).inverse())*geometric_jacobian.transpose()+Eigen::Matrix<double, 6, 6>::Identity()*0.000001;
        if (abs(geometric_jacobian.determinant()) < 0.0000001) ROS_INFO("Near singular configuration");

        // Quaternion error
        quaternion_error = quaternion_function(t, initial_quaternion, final_quaternion)*(quaternion_step.conjugate());
        
        // Inverse geometric jacobian input
        input_jacobian << position_velocity+Kp*(position_function(t, initial_point, final_point)-position_step), angular_velocity+Kq*(quaternion_error.vec());

        // Final computation
        q_derivative = inverse_geometric_jacobian*input_jacobian;
        q_step = q_step+q_derivative*delta_time;
        trajectory.conservativeResize(6, trajectory.cols()+1);
        trajectory.col(trajectory.cols()-1) = q_step;
    }

    return trajectory;
}
