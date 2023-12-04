/* My lib */
#include "robotic_project/kinematics.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

/* ROS lib */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h" 


void callback(const sensor_msgs::JointState msg) 
{
    /* angles values */
    Eigen::VectorXd q(8);
    for (int i=0; i<8; ++i) q(i)=msg.position[i];

    /* print the results */
    std::cout << "qe = " << std::endl << q << std::endl;

    /* arm theta angles => the arrangement of the theta angles are given by the subscriber information sequence */
    Vector6d arm_q(q(4), q(3), q(0), q(5), q(6), q(7));

    /* Compute the direct kinematics */
    UR5_t UR5(arm_q);
    std::cout << "pe = " << std::endl << UR5.__pe__ << std::endl;
    std::cout << "Re = " << std::endl << UR5.__Re__ << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5_joint_position_publisher");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/ur5/joint_states", 10, callback);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    ros::Rate rate(10);  // Adjust the publishing rate as needed (e.g., 10 Hz)

    while (ros::ok()) {
        std_msgs::Float64MultiArray jointCommand;
        jointCommand.data = {-1.32, -2.78, -0.1, -0.63, -0.57, 2.49, 0.55, 0.3};  // Set your desired joint positions

        pub.publish(jointCommand);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
}
