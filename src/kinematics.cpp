#include "robotic_project/kinematics.h"
#include "Eigen/Dense"
#include <cmath>

/* Constructor */
Kinematics::Kinematics(Vector6d q)
{
    direct_kinematics(q);
}

/* Direct Kinematic computation */
void Kinematics::direct_kinematics(Vector6d q) 
{
    /* Homogeneous Transformation matrix from frame 0 to 1 */
    Eigen::Matrix4d __HT01__ {
        {cos(q(0)) , -sin(q(0)) , 0 , 0},
        {sin(q(0)) , cos(q(0)) , 0 , 0},
        {0 , 0 , 1 , d(0)},
        {0 , 0 , 0 , 1}
    };

    /* Homogeneous Transformation matrix from frame 1 to 2 */
    Eigen::Matrix4d __HT12__ {
        {cos(q(1)) , -sin(q(1)) , 0 , 0},
        {0 , 0 , -1 , 0},
        {sin(q(1)) , cos(q(1)) , 0 , 0},
        {0 , 0 , 0 , 1}
    };

    /* Homogeneous Transformation matrix from frame 2 to 3 */
    Eigen::Matrix4d __HT23__ {
        {cos(q(2)) , -sin(q(2)) , 0 , a(1)},
        {sin(q(2)) , cos(q(2)) , 0 , 0},
        {0 , 0 , 1 , d(2)},
        {0 , 0 , 0 , 1}
    };

    /* Homogeneous Transformation matrix from frame 3 to 4 */
    Eigen::Matrix4d __HT34__ {
        {cos(q(3)) , -sin(q(3)) , 0 , a(2)},
        {sin(q(3)) , cos(q(3)) , 0 , 0},
        {0 , 0 , 1 , d(3)},
        {0 , 0 , 0 , 1}
    };

    /* Homogeneous Transformation matrix from frame 4 to 5 */
    Eigen::Matrix4d __HT45__ {
        {cos(q(4)) , -sin(q(4)) , 0 , 0},
        {0 , 0 , -1 , -d(4)},
        {sin(q(4)) , cos(q(4)) , 0 , 0},
        {0 , 0 , 0 , 1}
    };

    /* Homogeneous Transformation matrix from frame 5 to 6 */
    Eigen::Matrix4d __HT56__ {
        {cos(q(5)) , -sin(q(5)) , 0 , 0},
        {0 , 0 , 1 , d(5)},
        {-sin(q(5)) , -cos(q(5)) , 0 , 0},
        {0 , 0 , 0 , 1}
    };
    
    /* Homogeneous Transformation matrix from frame 6 to 0*/
    Eigen::Matrix4d __HT60__=__HT01__*__HT12__*__HT23__*__HT34__*__HT45__*__HT56__;

    /* Extract from the HT matrix the coordinates of the end-effector */
    __pe__=__HT60__.block<3,1>(0,3);

    /* Extract from the HT matrix the Rotation matrix */
    __Re__=__HT60__.block<3,3>(0,0);
}
