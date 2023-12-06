#include "robotic_project/kinematics.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

UR5_t::UR5_t() 
{
    /* set DH params */
    init_params();

    /* default values of q */
    __q__ <<  -0.32,-0.78, -2.56,-1.63, -1.57, 3.49;
}

/* Constructor */
UR5_t::UR5_t(Vector6d q)
{
    /* set DH params */
    init_params();

    /* Compute the initial pose */
    direct_kinematics();
}

void UR5_t::init_params()
{
    /* UR5 assignments for DH parameters */
    a << 0.0, -0.425, -0.3922, 0.0, 0.0, 0.0;
    d << 0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996;
    alpha << M_PI/2, 0.0, 0.0, M_PI/2, -M_PI/2, 0.0; 
}

void UR5_t::init_geometric_jacobian(Vector6d q)
{
    Vector6d J1(
        d(4)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + d(3)*cos(q(0)) - a(1)*cos(q(1))*sin(q(0)) - d(4)*sin(q(1) + q(2) + q(3))*sin(q(0)) - a(2)*cos(q(1))*cos(q(2))*sin(q(0)) + a(2)*sin(q(0))*sin(q(1))*sin(q(2)),
        d(4)*(cos(q(0))*cos(q(4)) + cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4))) + d(3)*cos(q(0)) - a(1)*cos(q(1))*sin(q(0)) - d(4)*sin(q(1) + q(2) + q(3))*sin(q(0)) - a(2)*cos(q(1))*cos(q(2))*sin(q(0)) + a(2)*sin(q(0))*sin(q(1))*sin(q(2)),
        0,
        0,
        0,
        1
    );

    Vector6d J2(
        -cos(q(0))*(a(2)*sin(q(1) + q(2)) + a(1)*sin(q(1)) + d(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - d(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
        -sin(q(0))*(a(2)*sin(q(1) + q(2)) + a(1)*sin(q(1)) + d(4)*(sin(q(1) + q(2))*sin(q(3)) - cos(q(1) + q(2))*cos(q(3))) - d(4)*sin(q(4))*(cos(q(1) + q(2))*sin(q(3)) + sin(q(1) + q(2))*cos(q(3)))),
        a(2)*cos(q(1) + q(2)) - (d(4)*sin(q(1) + q(2) + q(3) + q(4)))/2 + a(1)*cos(q(1)) + (d(4)*sin(q(1) + q(2) + q(3) - q(4)))/2 + d(4)*sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0
    );

    Vector6d J3(
        cos(q(0))*(d(4)*cos(q(1) + q(2) + q(3)) - a(2)*sin(q(1) + q(2)) + d(4)*sin(q(1) + q(2) + q(3))*sin(q(4))),
        sin(q(0))*(d(4)*cos(q(1) + q(2) + q(3)) - a(2)*sin(q(1) + q(2)) + d(4)*sin(q(1) + q(2) + q(3))*sin(q(4))),
        a(2)*cos(q(1) + q(2)) - (d(4)*sin(q(1) + q(2) + q(3) + q(4)))/2 + (d(4)*sin(q(1) + q(2) + q(3) - q(4)))/2 + d(4)*sin(q(1) + q(2) + q(3)),
        sin(q(0)),
        -cos(q(0)),
        0
    );

    Vector6d J4(
        d(4)*cos(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
        d(4)*sin(q(0))*(cos(q(1) + q(2) + q(3)) + sin(q(1) + q(2) + q(3))*sin(q(4))),
        d(4)*(sin(q(1) + q(2) + q(3) - q(4))/2 + sin(q(1) + q(2) + q(3)) - sin(q(1) + q(2) + q(3) + q(4))/2),
        sin(q(0)),
        -cos(q(0)),
        0
    );

    Vector6d J5(
        d(4)*cos(q(0))*cos(q(1))*cos(q(4))*sin(q(2))*sin(q(3)) - d(4)*cos(q(0))*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4)) - d(4)*sin(q(0))*sin(q(4)) + d(4)*cos(q(0))*cos(q(2))*cos(q(4))*sin(q(1))*sin(q(3)) + d(4)*cos(q(0))*cos(q(3))*cos(q(4))*sin(q(1))*sin(q(2)),
        d(4)*cos(q(0))*sin(q(4)) + d(4)*cos(q(1))*cos(q(4))*sin(q(0))*sin(q(2))*sin(q(3)) + d(4)*cos(q(2))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(3)) + d(4)*cos(q(3))*cos(q(4))*sin(q(0))*sin(q(1))*sin(q(2)) - d(4)*cos(q(1))*cos(q(2))*cos(q(3))*cos(q(4))*sin(q(0)),
        -d(4)*(sin(q(1) + q(2) + q(3) - q(4))/2 + sin(q(1) + q(2) + q(3) + q(4))/2),
        sin(q(1) + q(2) + q(3))*cos(q(0)),
        sin(q(1) + q(2) + q(3))*sin(q(0)),
        -cos(q(1) + q(2) + q(3))
    );

    Vector6d J6(
        0,
        0,
        0,
        cos(q(4))*sin(q(0)) - cos(q(1) + q(2) + q(3))*cos(q(0))*sin(q(4)),
        -cos(q(0))*cos(q(4)) - cos(q(1) + q(2) + q(3))*sin(q(0))*sin(q(4)),
        -sin(q(1) + q(2) + q(3))*sin(q(4))   
    );

    __GeometricJacobian__.col(0) = J1;
    __GeometricJacobian__.col(1) = J2;
    __GeometricJacobian__.col(2) = J3;
    __GeometricJacobian__.col(3) = J4;
    __GeometricJacobian__.col(4) = J5;
    __GeometricJacobian__.col(5) = J6;
}

void UR5_t::update_q(Vector6d new_q)
{
    /* set qes */
    q = new_q;

    /* change the current position vector __pe__ and rotation matrix __Re__ based on the q angles */
    direct_kinematics();
}

/* Direct Kinematic computation */
void UR5_t::direct_kinematics() 
{
    /* Calculate  transformation matrix from frame 6 to 0 */
    Eigen::Matrix4d __transformation_matrix__ = generate_transformation_matrix(a(0), alpha(0), d(0), q(0));
    for (int i=1; i<6; ++i) __transformation_matrix__ = __transformation_matrix__ * generate_transformation_matrix(a(i), alpha(i), d(i), q(i));
    
    /* Extract from the transformation matrix the coordinates of the end-effector */
    __pe__=__transformation_matrix__.block<3,1>(0,3);

    /* Extract from the transformation matrix the rotation matrix */
    __Re__=__transformation_matrix__.block<3,3>(0,0);
}

/* Compute the transformation matrix from frame i to frame i-1 */
Eigen::Matrix4d UR5_t::generate_transformation_matrix(double a, double alpha, double d, double theta)
{
    return Eigen::Matrix4d {
        {cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)},
        {sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)},
        {0, sin(alpha), cos(alpha), d},
        {0, 0, 0, 1}
    };
}

// void UR5_t::inverse_kinematics(Eigen::Vector3d P60)
// {
//     /* define the transformation matrix desired for the calculation of angles q*/
//     Eigen::Matrix4d T60 {
//         {1, 0, 0, P60(0)}, 
//         {0, 1, 0, P60(1)},
//         {0, 0, 1, P60(2)},
//         {0, 0, 0, 1}
//     };

//     /* define a generic vector V to compute the origin of frame 5 respect to frame 0, that is P50*/
//     Eigen::Vector4d V {0, 0, -d(5), 1};
//     Eigen::Vector4d P50 = T60*V;

//     /* compute ψ = atan2((P50)y, (P50)x) */
//     double psi = atan2(P50(1), P50(0));

//     /* distance between (P50)y and (P50)x */
//     double distance_P50x_P50y = hypot(P50(1), P50(0));

//     /* checking whether the P60 is reachable */
//     if (distance_P50x_P50y < d(3))
//     {
//         std::cout << "Position request in the unreachable cylinder" << std::endl;
//         return; 
//     }

//     /* φ = ± acos(d(3)/distance between (P50)y to (P50)x)*/
//     double phi1 = acos(d(3)/distance_P50x_P50y);
//     double phi2 = -phi1;

//     /* θ1 = ψ + φ + π/2 */
//     double theta1_1 = psi+phi1+M_PI/2;
//     double theta1_2 = psi+phi2+M_PI/2;

//     /* (P16)z = (P06)x sin(θ1) − (P06)y cos(θ1) */
//     double p61z_f_t1_1 = P60(0)*sin(theta1_1)-P60(1)*cos(theta1_1);
//     double p61z_f_t1_2 = P60(0)*sin(theta1_2)-P60(1)*cos(theta1_2);

//     /* θ5 = ± arccos(((P16)z - d(3)) / d6) */
//     double theta5_1_f_t1_1 = acos((p61z_f_t1_1-d(3))/d(5));
//     double theta5_2_f_t1_1 = -acos((p61z_f_t1_1-d(3))/d(5));
//     double theta5_1_f_t1_2 = acos((p61z_f_t1_2-d(3))/d(5));
//     double theta5_2_f_t1_2 = -acos((p61z_f_t1_2-d(3))/d(5));

//     /* compute the transformation matrix plotting the origin of frame 1 respect to frame 0 */
//     Eigen::Matrix4d T10_1_f_t1_1 = generate_transformation_matrix(a(0), alpha(0), d(0), theta1_1);
//     Eigen::Matrix4d T10_1_f_t1_2 = generate_transformation_matrix(a(0), alpha(0), d(0), theta1_2);

//     /* compute the two versions of the transformation matrix plotting the origin the frame 6 respect the frame 1 */
//     Eigen::Matrix4d T16_1_f_T10_1_f_t1_1 = (T10_1_f_t1_1.inverse()*T60).inverse();
//     Eigen::Matrix4d T16_1_f_T10_1_f_t1_2 = (T10_1_f_t1_2.inverse()*T60).inverse();

//     /* T16 = [[Xx Yx Zx Px],[Xy Yy Zy Py],[Xz Yz Zz Pz], [0 0 0 1]] */
//     double Zy_1 = T16_1_f_T10_1_f_t1_1(1,2);
//     double Zy_2 = T16_1_f_T10_1_f_t1_2(1,2);
//     double Zx_1 = T16_1_f_T10_1_f_t1_1(0,2);
//     double Zx_2 = T16_1_f_T10_1_f_t1_2(0,2);

//     /* θ6 = atan2(−zy/sin(θ5), zx/sin(θ5)) */
//     double theta6_1_f_t5_1_f_t1_1 = 0;
//     double theta6_1_f_t5_2_f_t1_1 = 0;
//     double theta6_1_f_t5_1_f_t1_2 = 0;
//     double theta6_1_f_t5_2_f_t1_2 = 0;

//     if (is_zero(sin(theta5_1_f_t1_1)) || (is_zero(Zy_1) && is_zero(Zx_1))) std::cout << "Singular configuration. Choosing arbitrary theta6" << std::endl;
//     else theta6_1_f_t5_1_f_t1_1 = atan2(((-Zy_1)/sin(theta5_1_f_t1_1)),(Zx_1/sin(theta5_1_f_t1_1)));

//     if (is_zero(sin(theta5_2_f_t1_1)) || (is_zero(Zy_1) && is_zero(Zx_1))) std::cout << "Singular configuration. Choosing arbitrary theta6" << std::endl;
//     else theta6_1_f_t5_2_f_t1_1 = atan2(((-Zy_1)/sin(theta5_2_f_t1_1)),(Zx_1/sin(theta5_2_f_t1_1)));

//     if (is_zero(theta5_1_f_t1_2) || (is_zero(Zy_2) && is_zero(Zx_2))) std::cout << "Singular configuration. Choosing arbitrary theta6" << std::endl;
//     else theta6_1_f_t5_1_f_t1_2 = atan2(((-Zy_2)/sin(theta5_2_f_t1_2)),(Zx_2/sin(theta5_2_f_t1_2)));

//     if (is_zero(theta5_2_f_t1_2) || (is_zero(Zy_2) && is_zero(Zx_2))) std::cout << "Singular configuration. Choosing arbitrary theta6" << std::endl;
//     else theta6_1_f_t5_2_f_t1_2 = atan2(((-Zy_2)/sin(theta5_2_f_t1_2)),(Zx_2/sin(theta5_2_f_t1_2)));

//     /* compute the transformation matrix plotting origin of frame 1 respect to frame 6 */
//     Eigen::Matrix4d T61_1_f_T16_1_f_T10_1_f_t1_1 = T16_1_f_T10_1_f_t1_1.inverse();
//     Eigen::Matrix4d T61_1_f_T16_1_f_T10_1_f_t1_2 = T16_1_f_T10_1_f_t1_2.inverse();

//     /* compute the transformation matrix from the origin of frame 5 respect to frame 4 */
//     Eigen::Matrix4d T54_1_f_t5_1_f_t1_1 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_1_f_t1_1);
//     Eigen::Matrix4d T54_1_f_t5_2_f_t1_1 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_2_f_t1_1);
//     Eigen::Matrix4d T54_1_f_t5_1_f_t1_2 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_2_f_t1_2);
//     Eigen::Matrix4d T54_1_f_t5_2_f_t1_2 = generate_transformation_matrix(a(4), alpha(4), d(4), theta5_2_f_t1_2);

//     /* compute the transformation matrix from the origin of frame 6 respect to frame 5 */
//     Eigen::Matrix4d T65_1_f_t6_1_f_t5_1_f_t1_1 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_1_f_t5_1_f_t1_1);
//     Eigen::Matrix4d T65_1_f_t6_1_f_t5_2_f_t1_1 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_1_f_t5_2_f_t1_1);
//     Eigen::Matrix4d T65_1_f_t6_1_f_t5_1_f_t1_2 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_1_f_t5_1_f_t1_2);
//     Eigen::Matrix4d T65_1_f_t6_1_f_t5_2_f_t1_2 = generate_transformation_matrix(a(5), alpha(5), d(5), theta6_1_f_t5_2_f_t1_2);

//     /* compute the transformation matrix from the origin of frame 4 respect to frame 1 */
//     Eigen::Matrix4d T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1 = T61_1_f_T16_1_f_T10_1_f_t1_1*((T54_1_f_t5_1_f_t1_1*T65_1_f_t6_1_f_t5_1_f_t1_1).inverse());
//     Eigen::Matrix4d T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1 = T61_1_f_T16_1_f_T10_1_f_t1_1*((T54_1_f_t5_2_f_t1_1*T65_1_f_t6_1_f_t5_2_f_t1_1).inverse());
//     Eigen::Matrix4d T41_1_f_T54_and_T65_1_f_t5_1_f_t1_2 = T61_1_f_T16_1_f_T10_1_f_t1_2*((T54_1_f_t5_1_f_t1_2*T65_1_f_t6_1_f_t5_1_f_t1_2).inverse());
//     Eigen::Matrix4d T41_1_f_T54_and_T65_1_f_t5_2_f_t1_2 = T61_1_f_T16_1_f_T10_1_f_t1_2*((T54_1_f_t5_2_f_t1_2*T65_1_f_t6_1_f_t5_2_f_t1_2).inverse());

//     /* compute the vector plotting the origin of frame 3 according to frame 1 */
//     V << 0.0, -d(3), 0.0, 1.0; Eigen::Vector4d P;
//     P = T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1*V; Eigen::Vector3d P31_1_f_T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1 << P(0), P(1), P(2);
//     P = T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1*V; Eigen::Vector3d P31_2_f_T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1 << P(0), P(1), P(2);
//     P = T41_1_f_T54_and_T65_1_f_t5_1_f_t1_2*V; Eigen::Vector3d P31_3_f_T41_1_f_T54_and_T65_1_f_t5_1_f_t1_2 << P(0), P(1), P(2);
//     P = T41_1_f_T54_and_T65_1_f_t5_2_f_t1_2*V; Eigen::Vector3d P31_4_f_T41_1_f_T54_and_T65_1_f_t5_2_f_t1_2 << P(0), P(1), P(2);

//     double C = (P31_1_f_T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1.squaredNorm()-pow(a(1),2.0)-pow(a(2),2.0))/(2*a(1)*a(2));
//     double theta_1_f_P31_1_f_T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1 = NULL;
//     double theta_2_f_P31_1_f_T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1 = NULL;

//     if (abs(C) > 1) std::cout << "Point out of the work space" << std::endl;
//     else 
//     {
//         theta_1_f_P31_1_f_T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1 = acos(C);
//         theta_2_f_P31_1_f_T41_1_f_T54_and_T65_1_f_t5_1_f_t1_1 = -acos(C);
//     }

//     C = (T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1.squaredNorm()-pow(a(1),2.0)-pow(a(2),2.0))/(2*a(1)*a(2));
//     double theta(2)_1_f_T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1 = NULL;
//     double theta(2)_2_f_T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1 = NULL;

//     if (abs(C) > 1) std::cout << "Point out of the work space" << std::endl;
//     else
//     {
//         theta(2)_1_f_T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1 = acos(C);
//         theta(2)_2_f_T41_1_f_T54_and_T65_1_f_t5_2_f_t1_1 = -acos(C);
//     }

//     // TODO: not finished
// }

/* check if a double value is effectively zero: If you want to compare a double to zero you should use this 
function due to precision error */
bool is_zero(double x)
{
    if (x > -0.0000001 && x < 0.0000001) return true;
    else return false;
}
