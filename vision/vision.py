#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from robotic_project.srv import ObtainBrickPose, ObtainBrickPoseResponse

import numpy as np
import torch
import tf

from os import path

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2

from pathlib import Path
from matplotlib import pyplot as plt
from torchvision.transforms import functional as F

import math

model = None
center_point = None
final_alpha = None
ready = False
brick_high = 0.89
brick_error = 0.001
value_error = 0.001
minimum_distance = 0.005
minimum_brick_lenght = 0.01
lenght_error =0.005
PI=3.14

def handle_obtain_brick_pose(req):
    # to compute with the model
    p = Pose()
    p.position.x = center_point[0]
    p.position.y = center_point[1]
    p.position.z = center_point[2]
    p.orientation.z = final_alpha
    
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
    # image = img[sliceBox]

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

    print(points_3d)

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

    pose_detection(data_world)

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

def tan_calculation(alpha1, alpha2, distance1, distance2):
    result = 0

    if (alpha1==-1 and alpha2==-1):
        print("TANGENT ERROR")
        result = 0
    elif (alpha1==-1):
        result = -alpha2
    elif (alpha2==-1):
        result = alpha1
    elif (alpha1<alpha2):
        result = alpha1
    else:
        result = -alpha2

    if(result==0):
        return result
    elif(result<0):
        if abs(distance2 - minimum_brick_lenght) <= lenght_error:
            return result
        else:
            return result + PI/2
    else:
        if abs(distance1 - minimum_brick_lenght) <= lenght_error:
            return result
        else:
            return result - PI/2
    
def three_points_selection(points):
    points_array=np.array(points)
    selected_points = points_array[abs(points_array[:,2] - brick_high) <=brick_error]

    min_x_index = np.argmin(selected_points[:,0])
    min_y_index = np.argmin(selected_points[:,1])
    max_y_index = np.argmax(selected_points[:,1])

    min_x_final_point = selected_points[min_x_index]
    min_y_value = selected_points[min_y_index, 1]
    max_y_value = selected_points[max_y_index, 1]

    min_y_selected_points = selected_points[abs(selected_points[:,1] - min_y_value) <= value_error]
    max_y_selected_points = selected_points[abs(selected_points[:,1] - max_y_value) <= value_error]


    min_y_final_index = np.argmin(min_y_selected_points[:,0])
    max_y_final_index = np.argmin(max_y_selected_points[:,0])

    min_y_final_point = min_y_selected_points[min_y_final_index]
    max_y_final_point = max_y_selected_points[max_y_final_index]
    
    final_points = np.array([min_y_final_point, min_x_final_point, max_y_final_point])
    return final_points

def pose_detection(points):
    three_points = three_points_selection(points)
    min_y = three_points[0]
    min_x = three_points[1]
    max_y = three_points[2]

    distance1 = math.dist(min_y,min_x)
    distance2 = math.dist(min_x,max_y)

    if(distance1>minimum_distance):
        alpha1 = math.atan2(min_y[0]-min_x[0],min_x[1]-min_y[1])
    else:
        alpha1 = -1

    if(distance2>minimum_distance):
        alpha2 = math.atan2(max_y[0]-min_x[0],max_y[1]-min_x[1])
    else:
        alpha2 = -1

    global final_alpha
    final_alpha = tan_calculation(alpha1, alpha2, distance1, distance2)

    print("Rotation")
    print(min_x)
    print(min_y)
    print(max_y)
    print(distance1)
    print(distance2)
    print(alpha1)
    print(alpha2)
    print(final_alpha)

if __name__ ==  '__main__':
    obtain_brick_pose_server()