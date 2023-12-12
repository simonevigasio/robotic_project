// Header
#include "robotic_project/kinematics.h"

// Eigen 
#include "Eigen/Dense"

// ROS
#include "ros/ros.h"

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

Eigen::Matrix4d directKin(Vector6d q)
{
    return T10f(q(0)) * T21f(q(1)) * T32f(q(2)) * T43f(q(3)) * T54f(q(4)) * T65f(q(5));
}

Eigen::Matrix<double, 8, 6> inverseKin(Eigen::Vector3d P60, Eigen::Matrix3d R60)
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

Eigen::Matrix3d fromEulerAnglesToRotationMatrix(Eigen::Vector3d eulerAngles)
{
    const double phi = eulerAngles(0);
    const double theta = eulerAngles(1);
    const double gamma = eulerAngles(2);
    
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

Eigen::Vector3d x(double t, Eigen::Vector3d initalPoint, Eigen::Vector3d finalPoint)
{
    const double nomalizedTime = t / durationTrajectory;
    if (nomalizedTime > 1) return finalPoint;
    else return (nomalizedTime * finalPoint) + ((1 - nomalizedTime) * initalPoint);
}

Eigen::Quaterniond slerp(double t, Eigen::Quaterniond initialQuaternion, Eigen::Quaterniond finalQuaternion)
{
    const double nomalizedTime = t / durationTrajectory;
    if (nomalizedTime > 1) return finalQuaternion;
    else return initialQuaternion.slerp(nomalizedTime, finalQuaternion);
}

Eigen::Matrix<double, 6, Eigen::Dynamic> inverseKinWithQuaternions(
    Vector6d q, 
    Eigen::Vector3d initialPoint,
    Eigen::Vector3d finalPoint, 
    Eigen::Quaterniond initialQuaternion, 
    Eigen::Quaterniond finalQuaternion 
)
{
    Vector6d qk; 
    Vector6d input;
    Vector6d qDot;

    Eigen::Matrix<double, 6, Eigen::Dynamic> trajectory;

    Eigen::Matrix4d transformationMatrixk;
    Eigen::Vector3d pointk;
    Eigen::Matrix3d rotationMatrixk;
    Eigen::Quaterniond quaternionk;

    Eigen::Vector3d angularVelocity;
    Eigen::Quaterniond quaternionVelocity;
    Eigen::Vector3d positionVelocity; 

    Eigen::Quaterniond quaternionError;
    Eigen::Matrix<double, 6, 6> jacobiank;
    Eigen::Matrix<double, 6, 6> inverseJacobiank;

    Eigen::Matrix3d Kp = Eigen::Matrix3d::Identity()*10;
    Eigen::Matrix3d Kq = Eigen::Matrix3d::Identity()*10;

    trajectory = qk = q;

    for (double t = dt; t < durationTrajectory; t += dt) 
    {
        transformationMatrixk = directKin(qk);
        pointk = transformationMatrixk.block(0, 3, 3, 1);
        rotationMatrixk = transformationMatrixk.block(0, 0, 3, 3);
        quaternionk = rotationMatrixk;

        positionVelocity = (x(t, initialPoint, finalPoint) - x(t - dt, initialPoint, finalPoint)) / dt;
        quaternionVelocity = slerp(t + dt, initialQuaternion, finalQuaternion) * slerp(t, initialQuaternion, finalQuaternion).conjugate(); 
	    angularVelocity = (quaternionVelocity.vec() * 2) / dt;

        jacobiank = jacobian(qk);
        inverseJacobiank = jacobiank.completeOrthogonalDecomposition().pseudoInverse() + (0.00001 * Eigen::Matrix<double, 6, 6>::Identity());
        if (abs(jacobiank.determinant()) < 0.0000001) ROS_INFO("Near singular configuration");

        quaternionError = slerp(t, initialQuaternion, finalQuaternion) * quaternionk.conjugate();
        
        input << 
            positionVelocity + (Kp * (x(t, initialPoint, finalPoint) - pointk)), 
            angularVelocity + (Kq * quaternionError.vec());

        qDot = inverseJacobiank * input;
        qk = qk + (qDot * dt);

        trajectory.conservativeResize(6, trajectory.cols() + 1);
        trajectory.col(trajectory.cols() - 1) = qk;
    }

    return trajectory;
}
