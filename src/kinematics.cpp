// Header
#include "robotic_project/kinematics.h"

// Eigen 
#include "Eigen/Dense"

// Standard
#include <iostream>
#include <cmath>

Eigen::Matrix4d T10f(double th1)
{
    return Eigen::Matrix4d {
        {cos(th1), -sin(th1), 0.0, 0.0},
        {sin(th1), cos(th1), 0.0, 0.0},
        {0.0, 0.0, 1.0, d(0)},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Eigen::Matrix4d T21f(double th2)
{
    return Eigen::Matrix4d {
        {cos(th2), -sin(th2), 0.0, 0.0},
        {0.0, 0.0, -1.0, 0.0},
        {sin(th2), cos(th2), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Eigen::Matrix4d T32f(double th3)
{
    return Eigen::Matrix4d {
        {cos(th3), -sin(th3), 0.0, a(1)},
        {sin(th3), cos(th3), 0.0, 0.0},
        {0.0, 0.0, 1.0, d(2)},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Eigen::Matrix4d T43f(double th4)
{
    return Eigen::Matrix4d {
        {cos(th4), -sin(th4), 0.0, a(2)},
        {sin(th4), cos(th4), 0.0, 0.0},
        {0.0, 0.0, 1.0, d(3)},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Eigen::Matrix4d T54f(double th5)
{
    return Eigen::Matrix4d {
        {cos(th5), -sin(th5), 0.0, 0.0},
        {0.0, 0.0, -1.0, -d(4)},
        {sin(th5), cos(th5), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Eigen::Matrix4d T65f(double th6)
{
    return Eigen::Matrix4d {
        {cos(th6), -sin(th6), 0.0, 0.0},
        {0.0, 0.0, 1.0, d(5)},
        {-sin(th6), -cos(th6), 0.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
}

Eigen::Matrix4d direct_kinematics(Vector6d q)
{
    return T10f(q(0)) * T21f(q(1)) * T32f(q(2)) * T43f(q(3)) * T54f(q(4)) * T65f(q(5));
}

Eigen::Matrix<double, 8, 6> inverse_kinematics(Eigen::Vector3d P60, Eigen::Matrix3d R60)
{
    Eigen::Matrix4d T60;
    T60.setZero();
    T60.block(0, 0, 3, 3) = R60;
    T60.block(0, 3, 3, 1) = P60;
    T60(3, 3) = 1;

    // Theta 1

    Eigen::Vector4d P50;
    P50 = T60 * Eigen::Vector4d(0, 0, -d(5), 1);
    double th1_1 = atan2(P50(1), P50(0)) + acos(d(3) / hypot(P50(1), P50(0))) + M_PI / 2;
    double th1_2 = atan2(P50(1), P50(0)) - acos(d(3) / hypot(P50(1), P50(0))) + M_PI / 2;

    // Theta 5

    double th5_1 = +acos((P60(0) * sin(th1_1) - P60(1) * cos(th1_1) - d(3)) / d(5));
    double th5_2 = -acos((P60(0) * sin(th1_1) - P60(1) * cos(th1_1) - d(3)) / d(5));
    double th5_3 = +acos((P60(0) * sin(th1_2) - P60(1) * cos(th1_2) - d(3)) / d(5));
    double th5_4 = -acos((P60(0) * sin(th1_2) - P60(1) * cos(th1_2) - d(3)) / d(5));

    // Theta 6

    Eigen::Matrix4d T06;
    T06 = T60.inverse();

    Eigen::Vector3d Xhat;
    Xhat = T06.block(0, 0, 3, 1);
    Eigen::Vector3d Yhat;
    Yhat = T06.block(0, 1, 3, 1);

    double th6_1 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1)) / sin(th5_1)), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1)) / sin(th5_1)));
    double th6_2 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1)) / sin(th5_2)), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1)) / sin(th5_2)));
    double th6_3 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_3)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_3)));
    double th6_4 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_4)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_4)));

    // Theta 3

    Eigen::Matrix4d T41m;
    Eigen::Vector3d P41_1;
    Eigen::Vector3d P41_2;
    Eigen::Vector3d P41_3;
    Eigen::Vector3d P41_4;
    double P41xz_1;
    double P41xz_2;
    double P41xz_3;
    double P41xz_4;

    T41m = T10f(th1_1).inverse() * T60 * T65f(th6_1).inverse() * T54f(th5_1).inverse();
    P41_1 = T41m.block(0, 3, 3, 1);
    P41xz_1 = hypot(P41_1(0), P41_1(2));

    T41m = T10f(th1_1).inverse() * T60 * T65f(th6_2).inverse() * T54f(th5_2).inverse();
    P41_2 = T41m.block(0, 3, 3, 1);
    P41xz_2 = hypot(P41_2(0), P41_2(2));

    T41m = T10f(th1_2).inverse() * T60 * T65f(th6_3).inverse() * T54f(th5_3).inverse();
    P41_3 = T41m.block(0, 3, 3, 1);
    P41xz_3 = hypot(P41_3(0), P41_3(2));

    T41m = T10f(th1_2).inverse() * T60 * T65f(th6_4).inverse() * T54f(th5_4).inverse();
    P41_4 = T41m.block(0, 3, 3, 1);
    P41xz_4 = hypot(P41_4(0), P41_4(2));

    double th3_1;

    if ((pow(P41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_1 = 0;
    }
    else if ((pow(P41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_1 = M_PI;
    }
    else
    {
        th3_1 = acos((pow(P41xz_1, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    double th3_2;

    if ((pow(P41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_2 = 0;
    }
    else if ((pow(P41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_2 = M_PI;
    }
    else
    {
        th3_2 = acos((pow(P41xz_2, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    double th3_3;

    if ((pow(P41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_3 = 0;
    }
    else if ((pow(P41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_3 = M_PI;
    }
    else
    {
        th3_3 = acos((pow(P41xz_3, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    double th3_4;

    if ((pow(P41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) > 1)
    {
        th3_4 = 0;
    }
    else if ((pow(P41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)) < -1)
    {
        th3_4 = M_PI;
    }
    else
    {
        th3_4 = acos((pow(P41xz_4, 2) - pow(a(1), 2) - pow(a(2), 2)) / (2 * a(1) * a(2)));
    }

    double th3_5 = -th3_1;
    double th3_6 = -th3_2;
    double th3_7 = -th3_3;
    double th3_8 = -th3_4;

    // Theta 2

    double th2_1 = atan2(-P41_1(2), -P41_1(0)) - asin((-a(2) * sin(th3_1)) / P41xz_1);
    double th2_2 = atan2(-P41_2(2), -P41_2(0)) - asin((-a(2) * sin(th3_2)) / P41xz_2);
    double th2_3 = atan2(-P41_3(2), -P41_3(0)) - asin((-a(2) * sin(th3_3)) / P41xz_3);
    double th2_4 = atan2(-P41_4(2), -P41_4(0)) - asin((-a(2) * sin(th3_4)) / P41xz_4);

    double th2_5 = atan2(-P41_1(2), -P41_1(0)) - asin((a(2) * sin(th3_1)) / P41xz_1);
    double th2_6 = atan2(-P41_2(2), -P41_2(0)) - asin((a(2) * sin(th3_2)) / P41xz_2);
    double th2_7 = atan2(-P41_3(2), -P41_3(0)) - asin((a(2) * sin(th3_3)) / P41xz_3);
    double th2_8 = atan2(-P41_4(2), -P41_4(0)) - asin((a(2) * sin(th3_4)) / P41xz_4);

    // Theta 4

    Eigen::Matrix4d T43m;
    Eigen::Vector3d Xhat43;
    T43m = T32f(th3_1).inverse() * T21f(th2_1).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_1).inverse() * T54f(th5_1).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_1 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_2).inverse() * T21f(th2_2).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_2).inverse() * T54f(th5_2).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_2 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_3).inverse() * T21f(th2_3).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_3).inverse() * T54f(th5_3).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_3 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_4).inverse() * T21f(th2_4).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_4).inverse() * T54f(th5_4).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_4 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_5).inverse() * T21f(th2_5).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_1).inverse() * T54f(th5_1).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_5 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_6).inverse() * T21f(th2_6).inverse() * T10f(th1_1).inverse() * T60 * T65f(th6_2).inverse() * T54f(th5_2).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_6 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_7).inverse() * T21f(th2_7).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_3).inverse() * T54f(th5_3).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_7 = atan2(Xhat43(1), Xhat43(0));

    T43m = T32f(th3_8).inverse() * T21f(th2_8).inverse() * T10f(th1_2).inverse() * T60 * T65f(th6_4).inverse() * T54f(th5_4).inverse();
    Xhat43 = T43m.block(0, 0, 3, 1);
    float th4_8 = atan2(Xhat43(1), Xhat43(0));

    // Configurations

    Eigen::Matrix<double, 8, 6> conf;
    conf << th1_1, th2_1, th3_1, th4_1, th5_1, th6_1,
        th1_1, th2_2, th3_2, th4_2, th5_2, th6_2,
        th1_2, th2_3, th3_3, th4_3, th5_3, th6_3,
        th1_2, th2_4, th3_4, th4_4, th5_4, th6_4,
        th1_1, th2_5, th3_5, th4_5, th5_1, th6_1,
        th1_1, th2_6, th3_6, th4_6, th5_2, th6_2,
        th1_2, th2_7, th3_7, th4_7, th5_3, th6_3,
        th1_2, th2_8, th3_8, th4_8, th5_4, th6_4;

    return conf;
}

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

Eigen::Matrix<double, 6, 6> jacobian(Vector6d q)
{
    Eigen::Matrix<double, 6, 6> J(6, 6);
    J.setZero();
    Vector6d J1(6, 1);
    J1 << d(4) * (cos(q(0)) * cos(q(4)) + cos(q(1) + q(2) + q(3)) * sin(q(0)) * sin(q(4))) + d(2) * cos(q(0)) + d(3) * cos(q(0)) - a(2) * cos(q(1) + q(2)) * sin(q(0)) - a(1) * cos(q(1)) * sin(q(0)) - d(4) * sin(q(1) + q(2) + q(3)) * sin(q(0)),
        d(4) * (cos(q(4)) * sin(q(0)) - cos(q(1) + q(2) + q(3)) * cos(q(0)) * sin(q(4))) + d(2) * sin(q(0)) + d(3) * sin(q(0)) + a(2) * cos(q(1) + q(2)) * cos(q(0)) + a(1) * cos(q(0)) * cos(q(1)) + d(4) * sin(q(1) + q(2) + q(3)) * cos(q(0)),
        0,
        0,
        0,
        1;
    Vector6d J2(6, 1);
    J2 << -cos(q(0)) * (a(2) * sin(q(1) + q(2)) + a(1) * sin(q(1)) + d(4) * (sin(q(1) + q(2)) * sin(q(3)) - cos(q(1) + q(2)) * cos(q(3))) - d(4) * sin(q(4)) * (cos(q(1) + q(2)) * sin(q(3)) + sin(q(1) + q(2)) * cos(q(3)))),
        -sin(q(0)) * (a(2) * sin(q(1) + q(2)) + a(1) * sin(q(1)) + d(4) * (sin(q(1) + q(2)) * sin(q(3)) - cos(q(1) + q(2)) * cos(q(3))) - d(4) * sin(q(4)) * (cos(q(1) + q(2)) * sin(q(3)) + sin(q(1) + q(2)) * cos(q(3)))),
        a(2) * cos(q(1) + q(2)) - (d(4) * sin(q(1) + q(2) + q(3) + q(4))) / 2 + a(1) * cos(q(1)) + (d(4) * sin(q(1) + q(2) + q(3) - q(4))) / 2 + d(4) * sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0;
    Vector6d J3(6, 1);
    J3 << cos(q(0)) * (d(4) * cos(q(1) + q(2) + q(3)) - a(2) * sin(q(1) + q(2)) + d(4) * sin(q(1) + q(2) + q(3)) * sin(q(4))),
        sin(q(0)) * (d(4) * cos(q(1) + q(2) + q(3)) - a(2) * sin(q(1) + q(2)) + d(4) * sin(q(1) + q(2) + q(3)) * sin(q(4))),
        a(2) * cos(q(1) + q(2)) - (d(4) * sin(q(1) + q(2) + q(3) + q(4))) / 2 + (d(4) * sin(q(1) + q(2) + q(3) - q(4))) / 2 + d(4) * sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0;
    Vector6d J4(6, 1);
    J4 << d(4) * cos(q(0)) * (cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3)) * sin(q(4))),
        d(4) * sin(q(0)) * (cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3)) * sin(q(4))),
        d(4) * (sin(q(1) + q(2) + q(3) - q(4)) / 2 + sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4)) / 2),
        sin(q(0)),
        -cos(q(0)),
        0;
    Vector6d J5(6, 1);
    J5 << -d(4) * sin(q(0)) * sin(q(4)) - d(4) * cos(q(1) + q(2) + q(3)) * cos(q(0)) * cos(q(4)),
        d(4) * cos(q(0)) * sin(q(4)) - d(4) * cos(q(1) + q(2) + q(3)) * cos(q(4)) * sin(q(0)),
        -d(4) * (sin(q(1) + q(2) + q(3) - q(4)) / 2 + sin(q(1) + q(2) + q(3) + q(4)) / 2),
        sin(q(1) + q(2) + q(3)) * cos(q(0)),
        sin(q(1) + q(2) + q(3)) * sin(q(0)),
        -cos(q(1) + q(2) + q(3));
    Vector6d J6(6, 1);
    J6 << 0,
        0,
        0,
        cos(q(4)) * sin(q(0)) - cos(q(1) + q(2) + q(3)) * cos(q(0)) * sin(q(4)),
        -cos(q(0)) * cos(q(4)) - cos(q(1) + q(2) + q(3)) * sin(q(0)) * sin(q(4)),
        -sin(q(1) + q(2) + q(3)) * sin(q(4));
    J << J1, J2, J3, J4, J5, J6;
    return J;
}

Eigen::Vector3d position_function(double t, Eigen::Vector3d initial_position, Eigen::Vector3d final_position)
{
    double nomalized_t = t/10.0;
    if (nomalized_t > 1) return final_position;
    else return nomalized_t*final_position+(1-nomalized_t)*initial_position;
}

Eigen::Quaterniond quaternion_function(double t, Eigen::Quaterniond initial_quaternian, Eigen::Quaterniond final_quaternian)
{
    double nomalized_t = t/10.0;
    if (nomalized_t > 1) return final_quaternian;
    else return initial_quaternian.slerp(nomalized_t, final_quaternian);
}

Eigen::Matrix<double, 6, Eigen::Dynamic> inverse_differential_kinematics_with_quaternions(Vector6d q, Eigen::Vector3d initial_point, Eigen::Quaterniond initial_quaternion, Eigen::Vector3d final_point, Eigen::Quaterniond final_quaternion)
{
    Vector6d q_step,input_jacobian, q_derivative;
    Eigen::Matrix<double, 6, Eigen::Dynamic> trajectory = q;
    Eigen::Matrix4d transformation_matrix_step;
    Eigen::Vector3d position_step, angular_velocity, position_velocity; 
    Eigen::Matrix3d rotation_matrix_step;
    Eigen::Quaterniond quaternion_step, quaternion_error, quaternion_velocity;
    Eigen::Matrix<double, 6, 6> jacobian_step, inverse_jacobian_step;
    double delta_time = 0.1; 
    double duration_trajectory = 10.0;

    q_step = q;

    for (double t=delta_time; t<duration_trajectory; t+=delta_time) 
    {
        Eigen::Matrix3d Kp = Eigen::Matrix3d::Identity()*10;
        Eigen::Matrix3d Kq = Eigen::Matrix3d::Identity()*10;

        // Current position and quaternion
        transformation_matrix_step = direct_kinematics(q_step);
        position_step = transformation_matrix_step.block<3,1>(0,3);
        std::cout << "Posizione" << std::endl;
        std::cout << position_step<< std::endl;
        rotation_matrix_step = transformation_matrix_step.block<3,3>(0,0);
        quaternion_step = rotation_matrix_step;

        // Desired velocities 
        position_velocity = (position_function(t, initial_point, final_point)-position_function(t-delta_time, initial_point, final_point))/delta_time;
        quaternion_velocity = (quaternion_function(t+delta_time, initial_quaternion, final_quaternion)*quaternion_function(t, initial_quaternion, final_quaternion).conjugate()); 
	    angular_velocity = (quaternion_velocity.vec()*2)/delta_time;

        // Jacobian computation
//        jacobian_step = ur5_geometric_jacobian(q_step);
        jacobian_step = jacobian(q_step);
        inverse_jacobian_step = jacobian_step.completeOrthogonalDecomposition().pseudoInverse() + 0.00001*(Eigen::Matrix<double, 6, 6>::Identity());
//  NON VA       inverse_jacobian_step = (jacobian_step.transpose)*(jacobian_step*(jacobian_step.transpose())).inverse();
        if (abs(jacobian_step.determinant()) < 0.0000001) std::cout << "Near singular configuration" << std::endl;

        // Quaternion error
        quaternion_error = quaternion_function(t, initial_quaternion, final_quaternion)*(quaternion_step.conjugate());
        
        // Inverse geometric jacobian input
        input_jacobian << position_velocity+Kp*(position_function(t, initial_point, final_point)-position_step), angular_velocity+Kq*(quaternion_error.vec());

        // Final computation
        q_derivative = inverse_jacobian_step*input_jacobian;
        q_step = q_step+q_derivative*delta_time;
        trajectory.conservativeResize(6, trajectory.cols()+1);
        trajectory.col(trajectory.cols()-1) = q_step;
    }

    return trajectory;
}
