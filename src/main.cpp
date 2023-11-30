/* Eigen Library */
#include "Eigen/Dense"

/* Standard Libraries */
#include <vector>
#include <iostream>

/* ROS Libraries */
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

typedef Eigen::Matrix<double, 1, 8> RowVector8d;

void callback(const sensor_msgs::JointState msg) {
    RowVector8d joints;
    for (int i=0; i<8; ++i) joints(i) = msg.position[i];
    std::cout << joints << std::endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ur5_joint_position_publisher");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/ur5/joint_states", 10, callback);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    ros::Rate rate(10);  // Adjust the publishing rate as needed (e.g., 10 Hz)

    while (ros::ok()) {
        std_msgs::Float64MultiArray jointCommand;
        jointCommand.data = {-1.32, -2.78, -1.1, -1.63, -1.57, 3.49, 0.55, 1.3};  // Set your desired joint positions

        pub.publish(jointCommand);
        ros::spinOnce();
        rate.sleep();
    }

    ros::spin();
}
