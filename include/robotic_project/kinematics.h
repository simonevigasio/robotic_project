#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__ 

// Eigen 
#include "../src/Eigen/Dense"

// ROS
#include "std_msgs/Float64MultiArray.h" 
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"

// Vector composed by 6 doubles
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class UR5
{
    public: 
        // Constructor 
        UR5();

    private:
        // ROS 
        ros::Publisher __pub;
        ros::Subscriber __sub;
        ros::NodeHandle __nh; 

        // Position and rotation matrix 
        Eigen::Vector3d __pe;
        Eigen::Matrix3d __Re;
        
        // DH parameters
        Vector6d __a, __d, __alpha, __q;

        // Dimension 
        const double __scalar_factor = 10.0;

        // Geometric Jacobian
        Eigen::Matrix<double, 6, 6> __geometric_jacobian;
        Eigen::Matrix<double, 6, 6> __inverse_geometric_jacobian;
        void compute_geometric_jacobian();

        // Callback of the subscriber
        void handle_movement(const sensor_msgs::JointState& msg); 

        // Trajectory parameters for time 
        bool __init = false;
        double __time_trajectory;
        const double __delta_time = 0.05;
        const double __duration_trajectory = 100.0;

        // Trajectory parameters for position and rotation
        Eigen::Vector3d __initial_position;
        Eigen::Vector3d __final_position;
        Eigen::Quaterniond __initial_quaternion;
        Eigen::Quaterniond __final_quaternion;

        // Trajectory error
        Eigen::Matrix3d __Kq;
        Eigen::Matrix3d __Kp;

        // Direct kinematric function 
        void direct_kinematics();

        // Generate T from i-1 to i given the DH parameters 
        Eigen::Matrix4d generate_transformation_matrix(double a, double alfa, double d, double theta);

        // Generate the quaternian from the rotation matrix
        Eigen::Quaterniond from_rotational_matrix_to_quaternion(Eigen::Matrix3d rotationMatrix);

        // Trajectory joints velocity 
        Eigen::VectorXd joints_velocity(Eigen::Vector3d position_velocity, Eigen::Vector3d position_error, Eigen::Vector3d angular_velocity, Eigen::Quaterniond quaternion_error); 
};

#endif
