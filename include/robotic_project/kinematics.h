#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

// Eigen 
#include "../src/Eigen/Dense"

// Vector composed by 6 doubles
typedef Eigen::Matrix<double, 6, 1> Vector6d;

// Dimensione factor
const double scalar_factor = 1.0;

// Correction parameters 
Eigen::Matrix3d Kp = Eigen::Matrix3d::Identity()*10; 
Eigen::Matrix3d Kq = Eigen::Matrix3d::Identity()*10; 

// time trajectory parameters
const double delta_time = 0.1;
const double duration_trajectory = 10.0;

// Static DH parameters 
Vector6d a(0.0, -0.425, -0.3922, 0.0, 0.0, 0.0);
Vector6d d(0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996);
Vector6d alpha(M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0);

// Read the q values from the UR5 
Vector6d read_q();

// Convertion
Eigen::Matrix3d from_euler_angles_to_rotation_matrix(Eigen::Vector3d euler_angles);

// Direct Kinematics
Eigen::Matrix4d generate_transformation_matrix(double a, double alpha, double d, double theta);
Eigen::Matrix4d direct_kinematics(Vector6d q); 

// Geometric Jacobian
Eigen::Matrix<double, 6, 6> my_ur5_geometric_jacobian(Vector6d q);
Eigen::Matrix<double, 6, 6> ur5_geometric_jacobian(Vector6d q);

// Generate a trajectory 
Eigen::Vector3d position_function(double t, Eigen::Vector3d initial_position, Eigen::Vector3d final_position);
Eigen::Quaterniond quaternion_function(double t, Eigen::Quaterniond initial_quaternian, Eigen::Quaterniond final_quaternian);
Eigen::Matrix<double, 6, Eigen::Dynamic> inverse_differential_kinematics_with_quaternions(Vector6d q, Eigen::Vector3d initial_point, Eigen::Quaterniond initial_quaternion, Eigen::Vector3d final_point, Eigen::Quaterniond final_quaternion);

#endif
