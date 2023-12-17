// Header
#include "robotic_project/kinematics.h"

// Eigen 
#include "Eigen/Dense"
#include "Eigen/QR"

// ROS
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "boost/shared_ptr.hpp"
#include "std_msgs/Float64MultiArray.h"

// Standard
#include <iostream>
#include <cmath>

M4d T10f(double th1)
{
    return M4d {
        {cos(th1) ,-sin(th1) ,0 ,0},
        {sin(th1) ,cos(th1) ,0 ,0},
        {0 ,0 ,1 ,d(0)},
        {0 ,0 ,0 ,1}
    };
}

M4d T21f(double th2)
{
    return M4d {
        {cos(th2) ,-sin(th2) ,0 ,0},
        {0 ,0 ,-1 ,0},
        {sin(th2) ,cos(th2) ,0 ,0},
        {0 ,0 ,0 ,1}
    };
}

M4d T32f(double th3)
{
    return M4d {
        {cos(th3) ,-sin(th3) ,0 ,a(1)},
        {sin(th3) ,cos(th3) ,0 ,0},
        {0 ,0 ,1 ,d(2)},
        {0 ,0 ,0 ,1}
    };
}

M4d T43f(double th4)
{
    return M4d {
        {cos(th4) ,-sin(th4) ,0 ,a(2)},
        {sin(th4) ,cos(th4) ,0 ,0},
        {0 ,0 ,1 ,d(3)},
        {0 ,0 ,0 ,1}
    };
}

M4d T54f(double th5)
{
    return M4d {
        {cos(th5) ,-sin(th5) ,0 ,0},
        {0 ,0 ,-1 ,-d(4)},
        {sin(th5) ,cos(th5) ,0 ,0},
        {0 ,0 ,0 ,1}
    };
}

M4d T65f(double th6)
{
    return M4d {
        {cos(th6) ,-sin(th6) ,0 ,0},
        {0 ,0 ,1 ,d(5)},
        {-sin(th6) ,-cos(th6) ,0 ,0},
        {0 ,0 ,0 ,1}
    };
}

M4d direct_kin(V6d js)
{
    return T10f(js(0)) * T21f(js(1)) * T32f(js(2)) * T43f(js(3)) * T54f(js(4)) * T65f(js(5));
}

InverseConfigurations inverse_kin(V3d P60, M3d R60)
{
    M4d T60;
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

    M4d T06;
    T06 = T60.inverse();

    V3d Xhat;
    Xhat = T06.block(0, 0, 3, 1);
    V3d Yhat;
    Yhat = T06.block(0, 1, 3, 1);

    double th6_1 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1)) / sin(th5_1)), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1)) / sin(th5_1)));
    double th6_2 = atan2(((-Xhat(1) * sin(th1_1) + Yhat(1) * cos(th1_1)) / sin(th5_2)), ((Xhat(0) * sin(th1_1) - Yhat(0) * cos(th1_1)) / sin(th5_2)));
    double th6_3 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_3)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_3)));
    double th6_4 = atan2(((-Xhat(1) * sin(th1_2) + Yhat(1) * cos(th1_2)) / sin(th5_4)), ((Xhat(0) * sin(th1_2) - Yhat(0) * cos(th1_2)) / sin(th5_4)));

    // Theta 3

    M4d T41m;
    V3d P41_1;
    V3d P41_2;
    V3d P41_3;
    V3d P41_4;
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

    M4d T43m;
    V3d Xhat43;
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

    InverseConfigurations conf;
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

M3d euler_to_rotation_matrix(V3d eu)
{
    // to do 
    return M3d::Identity();
}

Jacobian jacobian(V6d js)
{
    Jacobian J;
    J.setZero();
    V6d J1(6, 1);
    J1 << d(4) * (cos(js(0)) * cos(js(4)) + cos(js(1) + js(2) + js(3)) * sin(js(0)) * sin(js(4))) + d(2) * cos(js(0)) + d(3) * cos(js(0)) - a(2) * cos(js(1) + js(2)) * sin(js(0)) - a(1) * cos(js(1)) * sin(js(0)) - d(4) * sin(js(1) + js(2) + js(3)) * sin(js(0)),
        d(4) * (cos(js(4)) * sin(js(0)) - cos(js(1) + js(2) + js(3)) * cos(js(0)) * sin(js(4))) + d(2) * sin(js(0)) + d(3) * sin(js(0)) + a(2) * cos(js(1) + js(2)) * cos(js(0)) + a(1) * cos(js(0)) * cos(js(1)) + d(4) * sin(js(1) + js(2) + js(3)) * cos(js(0)),
        0,
        0,
        0,
        1;
    V6d J2(6, 1);
    J2 << -cos(js(0)) * (a(2) * sin(js(1) + js(2)) + a(1) * sin(js(1)) + d(4) * (sin(js(1) + js(2)) * sin(js(3)) - cos(js(1) + js(2)) * cos(js(3))) - d(4) * sin(js(4)) * (cos(js(1) + js(2)) * sin(js(3)) + sin(js(1) + js(2)) * cos(js(3)))),
        -sin(js(0)) * (a(2) * sin(js(1) + js(2)) + a(1) * sin(js(1)) + d(4) * (sin(js(1) + js(2)) * sin(js(3)) - cos(js(1) + js(2)) * cos(js(3))) - d(4) * sin(js(4)) * (cos(js(1) + js(2)) * sin(js(3)) + sin(js(1) + js(2)) * cos(js(3)))),
        a(2) * cos(js(1) + js(2)) - (d(4) * sin(js(1) + js(2) + js(3) + js(4))) / 2 + a(1) * cos(js(1)) + (d(4) * sin(js(1) + js(2) + js(3) - js(4))) / 2 + d(4) * sin(js(1) + js(2) + js(3)),
        sin(js(0)),
        -cos(js(0)),
        0;
    V6d J3(6, 1);
    J3 << cos(js(0)) * (d(4) * cos(js(1) + js(2) + js(3)) - a(2) * sin(js(1) + js(2)) + d(4) * sin(js(1) + js(2) + js(3)) * sin(js(4))),
        sin(js(0)) * (d(4) * cos(js(1) + js(2) + js(3)) - a(2) * sin(js(1) + js(2)) + d(4) * sin(js(1) + js(2) + js(3)) * sin(js(4))),
        a(2) * cos(js(1) + js(2)) - (d(4) * sin(js(1) + js(2) + js(3) + js(4))) / 2 + (d(4) * sin(js(1) + js(2) + js(3) - js(4))) / 2 + d(4) * sin(js(1) + js(2) + js(3)),
        sin(js(0)),
        -cos(js(0)),
        0;
    V6d J4(6, 1);
    J4 << d(4) * cos(js(0)) * (cos(js(1) + js(2) + js(3)) + sin(js(1) + js(2) + js(3)) * sin(js(4))),
        d(4) * sin(js(0)) * (cos(js(1) + js(2) + js(3)) + sin(js(1) + js(2) + js(3)) * sin(js(4))),
        d(4) * (sin(js(1) + js(2) + js(3) - js(4)) / 2 + sin(js(1) + js(2) + js(3)) - sin(js(1) + js(2) + js(3) + js(4)) / 2),
        sin(js(0)),
        -cos(js(0)),
        0;
    V6d J5(6, 1);
    J5 << -d(4) * sin(js(0)) * sin(js(4)) - d(4) * cos(js(1) + js(2) + js(3)) * cos(js(0)) * cos(js(4)),
        d(4) * cos(js(0)) * sin(js(4)) - d(4) * cos(js(1) + js(2) + js(3)) * cos(js(4)) * sin(js(0)),
        -d(4) * (sin(js(1) + js(2) + js(3) - js(4)) / 2 + sin(js(1) + js(2) + js(3) + js(4)) / 2),
        sin(js(1) + js(2) + js(3)) * cos(js(0)),
        sin(js(1) + js(2) + js(3)) * sin(js(0)),
        -cos(js(1) + js(2) + js(3));
    V6d J6(6, 1);
    J6 << 0,
        0,
        0,
        cos(js(4)) * sin(js(0)) - cos(js(1) + js(2) + js(3)) * cos(js(0)) * sin(js(4)),
        -cos(js(0)) * cos(js(4)) - cos(js(1) + js(2) + js(3)) * sin(js(0)) * sin(js(4)),
        -sin(js(1) + js(2) + js(3)) * sin(js(4));
    J << J1, J2, J3, J4, J5, J6;
    return J;
}

/*
    @brief compute the point in the instant t of the linear interpolation of x1 and x2

    @param[in] x1 and x2: 3D points
    @param[in] t: time
*/
V3d x(double t, V3d i_p, V3d f_p)
{
    const double n_t = t / d_path;
    if (n_t > 1) return f_p;
    else return (n_t * f_p) + ((1 - n_t) * i_p);
}

/*
    @brief compute the quaternion in the instant t of the slerp formed by q1 and q2

    @param[in] q1 and q2: initial quaternion and finale quaternion
    @param[in] t: time
*/
Qd slerp(double t, Qd q1, Qd q2)
{
    const double n_t = t / d_path;
    if (n_t > 1) return q2;
    else return q1.slerp(n_t, q2);
}

/*
    @brief compute the path of the robot accoring to the initial joints values

    @param[in] m_rt: robotic mesures of the joints and gripper
    @param[in] i_p, f_p: initial and final point of the path
    @param[in] i_q, f_q: initial and final quaternion of the path
*/
Path differential_inverse_kin_quaternions(V8d m_rt, V3d i_p, V3d f_p, Qd i_q, Qd f_q)
{
    /*
        @param gs: gripper actual opening
        @param js_k and js_dot_k: joints values in the instant k and its derivative dot in the same insatnt
    */
    V2d gs {m_rt(6), m_rt(7)};
    V6d js_k, js_dot_k; 

    /*
        @param fv: angular and positional velocities with the correction error
    */
    V6d fv;

    /*
        @param path: path of the robot
    */
    Path path;

    /*
        @param tm_k: transformation matrix in the instant k 
    */
    M4d tm_k;

    /*
        @param p_k: position of the robot in the instant k
    */
    V3d p_k;

    /*
        @param rm_k: rotation matrix of the robot in the instant k
    */
    M3d rm_k;

    /*
        @param q_k: quaternion related to the rotational matrix of the robot in the instant k
    */
    Qd q_k;

    /*
        @param av_k and pv_k: angular and positional velocities of the robot in the instant k
    */
    V3d av_k, pv_k;

    /*
        @param qv_k: quaternion velocity related to the angular velocity of the robot in the instant k
    */
    Qd qv_k;

    /*
        @param qerr_k: quaternion error of the rotational path (slerp) of the robot
    */
    Qd qerr_k;

    /*
        @param perr_k: positional error of the linear path (x) of the robot
    */
    V3d perr_k;

    /*
        @param j_k and invj_k: geometric jacobian and inverse geometric jacobian of the robot in the instant k
    */
    Jacobian j_k, invj_k;

    /*
        @param:
        --> Kp is for positional correction 
        --> Kq is for quaternion correction 
    */
    M3d Kp, Kq;
    Kp = Kq = M3d::Identity()*10;

    /*
        insert the starting point to the path
    */
    for (int i = 0; i < 6; ++i) js_k(i) = m_rt(i);
    path = insert_new_path_instance(path, js_k, gs);

    /*
        each delta time (dt) compute the joints state to insert into the path 
    */
    for (double t = dt; t < d_path; t += dt) 
    {
        /*
            compute the direct kinematics in the instant k 
        */
        tm_k = direct_kin(js_k);
        p_k = tm_k.block(0, 3, 3, 1);
        rm_k = tm_k.block(0, 0, 3, 3);
        q_k = rm_k;

        /*
            compute the velocities in the instant k
        */
        pv_k = (x(t, i_p, f_p) - x(t - dt, i_p, f_p)) / dt;
        qv_k = slerp(t + dt, i_q, f_q) * slerp(t, i_q, f_q).conjugate(); 
	    av_k = (qv_k.vec() * 2) / dt;

        /* 
            compute the jacobian and its inverse in the instant k
        */
        j_k = jacobian(js_k);
        invj_k = j_k.completeOrthogonalDecomposition().pseudoInverse() + (0.001 * Eigen::Matrix<double, 6, 6>::Identity());
        if (abs(j_k.determinant()) < 0.0000001) ROS_INFO("Near singular configuration");

        /*
            compute the errors in the path
        */
        qerr_k = slerp(t, i_q, f_q) * q_k.conjugate();
        perr_k = x(t, i_p, f_p) - p_k;
        
        /*
            compute the vector of the velocities composition with a parameter of correction
        */
        fv << pv_k + (Kp * perr_k), av_k + (Kq * qerr_k.vec());

        /*
            compute the joints state in the instant k
        */
        js_dot_k = invj_k * fv;
        js_k = js_k + (js_dot_k * dt);

        /*
            add it to the path
        */
        path = insert_new_path_instance(path, js_k, gs);
    }

    return path;
}

/*
    @brief insert a new joint state into the path of the robot

    @param[in] p 
    @def path of the robot

    @param[in] js
    @def joints state to insert into the path

    @param[in] gs
    @def gripper state of the opening
*/
Path insert_new_path_instance(Path p, V6d js, V2d gs)
{
    p.conservativeResize(p.rows() + 1, p.cols());
    p.row(p.rows() - 1) = V8d {js(0), js(1), js(2), js(3), js(4), js(5), gs(0), gs(1)};
    return p;
}

/*
    @brief open the gripper of the robot

    @param[in] m_rt: mesures of the robot (joints angles and gripper opening)
*/
Path open_gripper(V8d m_rt)
{
    /*
        @param sts: number of steps to open the gripper 
    */
    const int sts = 30; 
    const double dist_r = (0.3 - m_rt(6)) / sts;
    const double dist_l = (0.3 - m_rt(7)) / sts;

    Path p; 
    V6d js = arm_joint_state(m_rt);
    V2d gr {m_rt(6), m_rt(7)};

    for (int i = 1; i <= sts; ++i)
    {
        gr(0) += dist_r; 
        gr(1) += dist_l; 
        p = insert_new_path_instance(p, js, gr);
    }

    return p;
}

/*
    @brief close the gripper of the robot

    @param[in] m_rt: mesures of the robot (joints angles and gripper opening)
*/
Path close_gripper(V8d m_rt)
{
    /*
        @param sts: number of steps to close the gripper 
    */
    const int sts = 30;
    const double closing = -0.10; 
    const double gr_r_i = m_rt(6);
    const double gr_l_i = m_rt(7);
    const double dist_r = (closing - gr_r_i) / sts;
    const double dist_l = (closing - gr_l_i) / sts;

    Path p; 
    V6d js = arm_joint_state(m_rt);
    V2d gr {m_rt(6), m_rt(7)};

    for (int i = 1; i <= sts; ++i)
    {
        gr(0) = gr_r_i + dist_r * i; 
        gr(1) = gr_l_i + dist_l * i; 
        p = insert_new_path_instance(p, js, gr);
    }

    return p;
}

/*
    @brief change point in the world frame to base frame of the robot

    @param[in] xw: 3D point in the world frameFloat64MultiArray
*/
V3d world_to_base(V3d xw)
{
    /*
        @param T: transformation matrix used to change frame
    */
    M4d T;

    /*
        @param xb: point in the base robot frame
    */
    V3d xb; 

    /*
        param xt: temp point used for the computation
    */
    V4d xt;

    T << 1.0, 0.0, 0.0, 0.5,
        0.0, -1.0, 0.0, 0.35,
        0.0, 0.0, -1.0, 1.75,
        0.0, 0.0, 0.0, 1.0;

    xt = T.inverse() * V4d(xw(0), xw(1), xw(2), 1.0);
    xb << xt(0), xt(1), xt(2);
    return xb;
}

/*
    @brief read the measures of the robot (joints, gripper)
*/
V8d read_robot_measures()
{
    boost::shared_ptr<sensor_msgs::JointState const> joints_state_msg;
    joints_state_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
    V8d js;
    for (int i = 0; i < 8; ++i) js(i) = joints_state_msg->position[i];
    return V8d {js(4), js(3), js(0), js(5), js(6), js(7), js(1), js(2)};
}

/*
    @brief get from the robot measures only the joints state

    @param[in] m_rt: robot measures 
*/
V6d arm_joint_state(V8d m_rt)
{
    return V6d {m_rt(0), m_rt(1), m_rt(2), m_rt(3), m_rt(4), m_rt(5)};
}

/*
    @brief apply the path desired

    @param[in] mv: desired path to apply for the robot
*/
void apply_movement(Path mv, ros::Publisher pub)
{
    ros::Rate loop_rate(10);
    for (int i = 0; i < mv.rows(); ++i)
    {
        V8d js = V8d {mv(i, 0), mv(i, 1), mv(i, 2), mv(i, 3), mv(i, 4), mv(i, 5), mv(i, 6), mv(i, 7)};
        std_msgs::Float64MultiArray jsm;
        jsm.data.resize(8);
        for (int j = 0; j < 8; j++)
        {
            jsm.data[j] = js(j);
        }
        pub.publish(jsm);
        loop_rate.sleep();
    }
}

// void move_brick(V2d xy_brick, V3d eu_brick, ros::Publisher pub)
// {

// }

void move(V3d f_p, V3d f_eu, ros::Publisher pub)
{
    V8d m_rt = read_robot_measures();
    V6d js = arm_joint_state(m_rt);

    M4d i_tm = direct_kin(js);
    M3d i_rm= i_tm.block(0, 0, 3, 3);
    V3d i_p = i_tm.block(0, 3, 3, 1);
    Qd i_q(i_rm);

    M3d f_rm = M3d::Identity();
    Qd f_q(f_rm);

    Path p = differential_inverse_kin_quaternions(m_rt, i_p, f_p, i_q, f_q);
    apply_movement(p, pub);
}

// void toggle_gripper(ros::Publisher pub)
// {
//     m_rt = read_robot_measures();
//     js = arm_joint_state(m_rt);

//     if (m_rt(6) > 0 && m_rt(7) > 0) Path p = close_gripper(m_rt);
//     else if (m_rt(6) <= 0 && m_rt(7) <= 0) Path p = open_gripper(M_rt);
//     else ROS_INFO("Inconsistent opening of the gripper");

//     apply_movement(p, pub);
// }
