#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from robotic_project.srv import ObtainBrickPose, ObtainBrickPoseResponse

import numpy as np
import torch
import tf

from os import path

import cv2 as cv
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2

from pathlib import Path
from matplotlib import pyplot as plt
from torchvision.transforms import functional as F

model = None
center_point = None
ready = False

def handle_obtain_brick_pose(req):
    # to compute with the model
    p = Pose()
    p.position.x = center_point[0]
    p.position.y = center_point[1]
    p.position.z = center_point[2]
    print('Returning brick location')
    return ObtainBrickPoseResponse(p)

def detection(msg: Image) -> None:
    # convert received image (bgr8 format) to a cv2 image
    img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    # loading yolov5 model
    global model
    result = model(img)

    lego_list= []
    bboxes = result.pandas().xyxy[0].to_dict(orient="records")
    for bbox in bboxes:
        name = bbox['name']
        conf = bbox['confidence']
        x1 = int(bbox['xmin'])
        y1 = int(bbox['ymin'])
        x2 = int(bbox['xmax'])
        y2 = int(bbox['ymax'])
        lego_list.append((name, conf, x1, y1, x2, y2))

    sliceBox = slice(y1, y2) , slice(x1,  x2)
    #points = img[sliceBox]

    points_2D = []

    for x in range(x1, x2+1):
            for y in range(y1, y2+1):
                points_2D.append([x, y])

    point_cloud2_msg = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)

    zed_points = []
    multiple_points = all(isinstance(item, list) or isinstance(item, tuple) for item in points_2D)

    if multiple_points:

        for point in list(points_2D):
            zed_points.append([int(coordinate) for coordinate in point])
    
    else:
        zed_points = list(points_2D)
        zed_points = [int(coordinate) for coordinate in zed_points]

    points_3d = point_cloud2.read_points(point_cloud2_msg, field_names=['x','y','z'], skip_nans=False, uvs=zed_points if multiple_points else [zed_points])

    Ry = np.array([[ 0.     , -0.49948,  0.86632],[-1.     ,  0.     ,  0.     ],[-0.     , -0.86632, -0.49948]])
    pos_zed = np.array([-0.4 ,  0.59,  1.4 ])

    zed_points = []
    for point in points_3d:
        zed_points.append(point[:3])

    data_world = []
    for point in zed_points:
        point = Ry.dot(point) + pos_zed
        point = np.array(point)
        data_world.append(point)
    
    global center_point
    center_point = np.mean(data_world, axis=0)

    global ready
    ready = True
    
def obtain_brick_pose_server():
    rospy.init_node('vision_node')

    global model

    path_yolo = "/home/vegas/ros_ws/src/robotic_project/vision/yolov5"
    path_vision = "/home/vegas/ros_ws/src/robotic_project/vision"
    path_weigths = path.join(path_vision, 'best.pt')

    model = torch.hub.load(path_yolo, "custom", path_weigths, source='local')

    rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, callback = detection, queue_size=1)
    point_cloud2_msg = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)

    s = rospy.Service('obtain_brick_pose', ObtainBrickPose, handle_obtain_brick_pose)

    global ready 
    while (not ready): None

    print("the vision is ready to deliver the block position.")

    rospy.spin()

if __name__ ==  '__main__':
    obtain_brick_pose_server()