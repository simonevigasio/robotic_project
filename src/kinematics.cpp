#include "robotic_project/kinematics.h"
#include "Eigen/Dense"
#include <cmath>

/* Constructor */
UR5_t::UR5_t(Vector6d q)
{
    /* UR5 assignments for DH parameters */
    a << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
    d << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
    alfa << M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0; 

    /* Compute the initial pose */
    direct_kinematics(q);
}

/* Direct Kinematic computation */
void UR5_t::direct_kinematics(Vector6d q) 
{
    /* Calculate  transformation matrix from frame 6 to 0 */
    Eigen::Matrix4d __transformation_matrix__ = generate_transformation_matrix(a(0), alfa(0), d(0), q(0));
    for (int i=1; i<6; ++i) __transformation_matrix__ = __transformation_matrix__ * generate_transformation_matrix(a(i), alfa(i), d(i), q(i));
    
    /* Extract from the transformation matrix the coordinates of the end-effector */
    __pe__=__transformation_matrix__.block<3,1>(0,3);

    /* Extract from the transformation matrix the rotation matrix */
    __Re__=__transformation_matrix__.block<3,3>(0,0);
}

/* Compute the transformation matrix from frame i to i-1 */
Eigen::Matrix4d UR5_t::generate_transformation_matrix(double a, double alfa, double d, double theta)
{
    return Eigen::Matrix4d {
        {cos(theta), -sin(theta)*cos(alfa), sin(theta)*sin(alfa), a*cos(theta)},
        {sin(theta), cos(theta)*cos(alfa), -cos(theta)*sin(alfa), a*sin(theta)},
        {0, sin(alfa), cos(alfa), d},
        {0, 0, 0, 1}
    };
}
