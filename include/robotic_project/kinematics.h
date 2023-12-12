#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

// Eigen
#include "../src/Eigen/Dense"

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Vector6d a(0.0, -0.425, -0.3922, 0.0, 0.0, 0.0);
Vector6d d(0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996);
Vector6d alpha(M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0);

Eigen::Matrix4d T10f(double th1);
Eigen::Matrix4d T21f(double th2);
Eigen::Matrix4d T32f(double th3);
Eigen::Matrix4d T43f(double th4);
Eigen::Matrix4d T54f(double th5);
Eigen::Matrix4d T65f(double th6);
Eigen::Matrix4d direct_kinematics(Vector6d q);
Eigen::Matrix<double, 8, 6> inverse_kinematics(Eigen::Vector3d P60, Eigen::Matrix3d R60);
Eigen::Matrix<double, 6, 6> jacobian(Vector6d q);

Eigen::Matrix3d from_euler_angles_to_rotation_matrix(Eigen::Vector3d euler_angles);
Eigen::Vector3d position_function(double t, Eigen::Vector3d initial_position, Eigen::Vector3d final_position);
Eigen::Quaterniond quaternion_function(double t, Eigen::Quaterniond initial_quaternian, Eigen::Quaterniond final_quaternian);
Eigen::Matrix<double, 6, Eigen::Dynamic> inverse_differential_kinematics_with_quaternions(Vector6d q, Eigen::Vector3d initial_point, Eigen::Quaterniond initial_quaternion, Eigen::Vector3d final_point, Eigen::Quaterniond final_quaternion);

#endif
