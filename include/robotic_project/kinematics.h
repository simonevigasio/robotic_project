#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

#include "../src/Eigen/Dense"
#include "std_msgs/Float64MultiArray.h" 
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

/* Vector composed by 6 doubles */
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/* Kinematics class incorporates the kinematic status of the ur5 robot given its six DH angles configuration */
class UR5
{
    public: 
        /* Default Constructor */
        UR5(ros::NodeHandle nh);

        /* Constructor */
        UR5(Vector6d q, ros::NodeHandle nh);

        /* Where the end-effector stands in comparison with the frame 0 */
        Eigen::Vector3d __pe__;

        /* Rotation matrix of the end-effector in comparison with the frame 0 */
        Eigen::Matrix3d __Re__;

        /* currect angle values */
        Vector6d __q__;

        /* jacobian */
        Eigen::Matrix<double, 6, 6> __geometric_jacobian__;

        /* inverse jacobian */
        Eigen::Matrix<double, 6, 6> __inverse_geometric_jacobian__;

        /* motion function */
        void motion_plan(Eigen::Vector3d final_point, Eigen::Matrix3d final_rotation_matrix);

    private:
        /* DH params of UR5 */
        Vector6d a, d, alpha;

        /* ROS  */
        ros::Publisher pub;
        ros::Subscriber sub;

        /* init ROS evairoment */
        void init_ROS(ros::NodeHandle nh);

        /* set the q values */
        void update_q(const sensor_msgs::JointState msg); 

        /* set DH params */
        void init_params();

        /* change the value of direct kinematics and geometric jacobian based on the new values of __q__*/
        void update_position();

        /* Direct Kinematics colputation of __pe__ and __Re__ */
        void direct_kinematics();

        /* set up geometric jacobian */
        void compute_geometric_jacobian();

        /* Compute the transformation matrix from i to i-1 */
        Eigen::Matrix4d generate_transformation_matrix(double a, double alfa, double d, double theta);


        // new functs

        bool __set_initial_q__ = false;

        const double __delta_time__= 0.25;

        Eigen::Vector3d __initial_position__;
        Eigen::Vector3d __final_position__;

        Eigen::Quaterniond __initial_quaternion__;
        Eigen::Quaterniond __final_quaternion__;

        Eigen::Matrix3d __Kq__;
        Eigen::Matrix3d __Kp__;


        Eigen::Quaterniond from_rotational_matrix_to_quaternion(Eigen::Matrix3d rotationMatrix);
        Eigen::Vector3d compute_desired_cartesian_position(double normalized_time);
        Eigen::Quaterniond compute_desired_quaternion(double normalized_time);
        Eigen::Vector3d calculate_angular_velocity(double normalized_time);
        Eigen::Vector3d calculate_cartesian_position_velocity(double normalized_time);
        Eigen::Quaterniond compute_quaternion_error(double normalized_time);
        Eigen::Vector3d compute_cartesian_position_error(double normalized_time);
        Eigen::VectorXd joints_velocity_calculation(double normalized_time);
        Eigen::VectorXd calculate_next_joints(double normalized_time);    
        double time_normalization(double time, double max_time);
};

#endif
