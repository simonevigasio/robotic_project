#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from robotic_project.srv import ObtainBrickPose, ObtainBrickPoseResponse

def handle_obtain_brick_pose(req):
    p = Pose()
    p.position.x = 0.640031
    p.position.y = 0.599445
    p.position.z = 0.869925
    print('Returning brick location', p.position.x, p.position.y, p.position.z)
    return ObtainBrickPoseResponse(p)

def obtain_brick_pose_server():
    rospy.init_node('test_node')
    s = rospy.Service('obtain_brick_pose', ObtainBrickPose, handle_obtain_brick_pose)
    print("Ready to give the brick pose.")
    rospy.spin()


if __name__ == '__main__':
    obtain_brick_pose_server()