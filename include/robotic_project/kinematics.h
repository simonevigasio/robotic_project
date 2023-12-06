#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

#include "../src/Eigen/Dense"

/* Vector composed by 6 doubles */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/* Kinematics class incorporates the kinematic status of the ur5 robot given its six DH angles configuration */
class UR5_t
{
    public: 
        /* Default Constructor */
        UR5_t();

        /* Constructor */
        UR5_t(Vector6d q);

        /* Where the end-effector stands in comparison with the frame 0 */
        Eigen::Vector3d __pe__;

        /* Rotation matrix of the end-effector in comparison with the frame 0 */
        Eigen::Matrix3d __Re__;

        /* currect angle values */
        Vector6d __q__;

        /* jacobian */
        Eigen::Matrix<double, 6, 6> __GeometricJacobian__;

        /* set the q values */
        void update_q(Vector6d new_q); 

        /* set up geometric jacobian */
        void init_geometric_jacobian(Vector6d q);

        /* Inverse Kinematics */
        /* void inverse_kinematics(Eigen::Vector3d P60); */
    private:
        /* DH params of UR5 */
        Vector6d a, d, alpha;

        /* set DH params */
        void init_params();

        /* Direct Kinematics colputation of __pe__ and __Re__ */
        void direct_kinematics();

        /* Compute the transformation matrix from i to i-1 */
        Eigen::Matrix4d generate_transformation_matrix(double a, double alfa, double d, double theta);
};

/* bool is_zero(double x); */

#endif
