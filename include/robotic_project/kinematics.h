#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

// Eigen
#include "../src/Eigen/Dense"

// ROS
#include "ros/ros.h"

typedef Eigen::Matrix<double, 6, 1> V6d;
typedef Eigen::Matrix<double, 8, 1> V8d;
typedef Eigen::Vector4d V4d;
typedef Eigen::Vector3d V3d;
typedef Eigen::Vector2d V2d;

typedef Eigen::Quaterniond Qd;

typedef Eigen::Matrix<double, Eigen::Dynamic, 8> Path;
typedef Eigen::Matrix<double, 6, 6> Jacobian;
typedef Eigen::Matrix<double, 8, 6> InverseConfigurations;

typedef Eigen::Matrix3d M3d;
typedef Eigen::Matrix4d M4d;

V6d a(0.0, -0.425, -0.3922, 0.0, 0.0, 0.0);
V6d d(0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996);
V6d alpha(M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0);

const double dt = 0.1; 
const double d_path = 10.0;

M4d T10f(double th1);
M4d T21f(double th2);
M4d T32f(double th3);
M4d T43f(double th4);
M4d T54f(double th5);
M4d T65f(double th6);
M4d direct_kin(V6d js);
InverseConfigurations inverse_kin(V3d P60, M3d R60);
Jacobian jacobian(V6d q);

V3d x(double t, V3d x1, V3d x2);
Qd slerp(double t, Qd q1, Qd q2);

V3d world_to_base(V3d xw);
M3d euler_to_rotation_matrix(V3d eu);

Path differential_inverse_kin_quaternions(V8d m_rt, V3d i_p, V3d f_p, Qd i_q, Qd f_q);
Path insert_new_path_instance(Path p, V6d js, V2d gs);

V8d read_robot_measures();
V6d arm_joint_state(V8d m_rt);
void apply_movement(Path mv, ros::Publisher pub);

#endif
