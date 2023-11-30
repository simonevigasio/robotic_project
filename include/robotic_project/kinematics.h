#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

#include "../src/Eigen/Dense"

/* Vector composed of 6 double numbers */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/* DH parameters */
Vector6d a(0.0, -0.425, -0.3922, 0.0, 0.0, 0.0);
Vector6d d(0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996);

/* Kinematics class incorporates the kinematic status of the ur5 robot given its six DH angles configuration */
class Kinematics 
{
    public:  
        /* Constructor */
        Kinematics(Vector6d q);

        /* Where the end-effector stands in comparison with the frame 0 */
        Eigen::Vector3d __pe__;

        /* Rotation matrix of the end-effector in comparison with the frame 0 */
        Eigen::Matrix3d __Re__;

        /* Direct Kinematics colputation of __pe__ and __Re__ */
        void direct_kinematics(Vector6d q);
};

#endif
