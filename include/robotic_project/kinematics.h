#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

#include "../src/Eigen/Dense"

/* Vector composed by 6 doubles */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/* Kinematics class incorporates the kinematic status of the ur5 robot given its six DH angles configuration */
class UR5_t
{
    public:  
        /* Constructor */
        UR5_t(Vector6d q);

        /* Where the end-effector stands in comparison with the frame 0 */
        Eigen::Vector3d __pe__;

        /* Rotation matrix of the end-effector in comparison with the frame 0 */
        Eigen::Matrix3d __Re__;

    private: 
        /* DH params of UR5 */
        Vector6d a, d, alfa;

        /* Direct Kinematics colputation of __pe__ and __Re__ */
        void direct_kinematics(Vector6d q);

        /* Compute the transformation matrix from i to i-1 */
        Eigen::Matrix4d generate_transformation_matrix(double a, double alfa, double d, double theta);
};

#endif
