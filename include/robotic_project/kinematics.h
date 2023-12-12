#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

// Eigen
#include "../src/Eigen/Dense"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Vector6d a(0.0, -0.425, -0.3922, 0.0, 0.0, 0.0);
Vector6d d(0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996);
Vector6d alpha(M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0);

const double dt = 0.1; 
const double durationTrajectory = 10.0;

Eigen::Matrix4d T10f(double th1);
Eigen::Matrix4d T21f(double th2);
Eigen::Matrix4d T32f(double th3);
Eigen::Matrix4d T43f(double th4);
Eigen::Matrix4d T54f(double th5);
Eigen::Matrix4d T65f(double th6);
Eigen::Matrix4d directKin(Vector6d q);
Eigen::Matrix<double, 8, 6> inverseKin(Eigen::Vector3d P60, Eigen::Matrix3d R60);
Eigen::Matrix<double, 6, 6> jacobian(Vector6d q);
Eigen::Matrix3d fromEulerAnglesToRotationMatrix(Eigen::Vector3d eulerAngles);
Eigen::Vector3d x(double t, Eigen::Vector3d initalPoint, Eigen::Vector3d finalPoint);
Eigen::Quaterniond slerp(double t, Eigen::Quaterniond initialQuaternion, Eigen::Quaterniond finalQuaternion);
Eigen::Matrix<double, 6, Eigen::Dynamic> inverseKinWithQuaternions(
    Vector6d q, 
    Eigen::Vector3d initialPoint,
    Eigen::Vector3d finalPoint, 
    Eigen::Quaterniond initialQuaternion, 
    Eigen::Quaterniond finalQuaternion 
);

#endif
