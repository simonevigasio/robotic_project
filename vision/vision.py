#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from robotic_project.srv import ObtainBrickPose, ObtainBrickPoseResponse

import numpy as np
import torch
import tf

from os import path

#import open3d as o3d

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

brick_short_side = None
brick_long_side = None
brick_high = None

table_high = 0.88
brick_high_detection = None
brick_error = 0.001
value_error = 0.001

minimum_distance = 0.005
lenght_error = 0.005

brick_center_point = None

PI=3.14

brick_value = {
        'X1-Y1-Z2': [0.03, 0.03 , 0.02],
        'X1-Y2-Z1': [0.03, 0.06 , 0.01],
        'X1-Y2-Z2-CHAMFER': [0.03, 0.06 , 0.02],
        'X1-Y2-Z2-TWINFILLET': [0.03, 0.06 , 0.02],
        'X1-Y2-Z2': [0.03, 0.06 , 0.02],
        'X1-Y3-Z2-FILLET': [0.01, 0.09 , 0.02],
        'X1-Y3-Z2': [0.03, 0.09 , 0.02],
        'X1-Y4-Z1': [0.03, 0.12 , 0.01],
        'X1-Y4-Z2': [0.03, 0.12 , 0.02],
    }

brick_value_WRONG = {
        'X1-Y1-Z2': [0.01, 0.01 , 0.02],
        'X1-Y2-Z1': [0.01, 0.02 , 0.01],
        'X1-Y2-Z2-CHAMFER': [0.01, 0.02 , 0.02],
        'X1-Y2-Z2-TWINFILLET': [0.01, 0.02 , 0.02],
        'X1-Y2-Z2': [0.01, 0.02 , 0.02],
        'X1-Y3-Z2-FILLET': [0.01, 0.03 , 0.02],
        'X1-Y3-Z2': [0.01, 0.03 , 0.02],
        'X1-Y4-Z1': [0.01, 0.04 , 0.01],
        'X1-Y4-Z2': [0.01, 0.04 , 0.02],
    }

def handle_obtain_brick_pose(req):
    # to compute with the model
    p = Pose()
    p.position.x = brick_center_point[0]
    p.position.y = brick_center_point[1]
    p.position.z = brick_center_point[2]
    p.orientation.z = final_alpha
    
    print('Returning brick location')
    return ObtainBrickPoseResponse(p)

def get_brick_value(name):
    value = np.array(brick_value[name])

    global brick_short_side
    brick_short_side = value[0]

    global brick_long_side
    brick_long_side = value[1]

    global brick_high
    brick_high = value[2]

    global brick_high_detection
    brick_high_detection = table_high + brick_high/2

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
        get_brick_value(name)
        conf = bbox['confidence']
        x1 = int(bbox['xmin'])
        y1 = int(bbox['ymin'])
        x2 = int(bbox['xmax'])
        y2 = int(bbox['ymax'])
        lego_list.append((name, conf, x1, y1, x2, y2))



    sliceBox = slice(y1, y2) , slice(x1,  x2)
    image = img[sliceBox]
        
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #come image metti i punti della boundery box
    color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255)], # Hue range: 0-10
            'green': [(36, 50, 50), (70, 255, 255)], # Hue range: 36-70
            'blue': [(90, 50, 50), (130, 255, 255)], # Hue range: 90-130
            'yellow': [(20, 50, 50), (35, 255, 255)], # Hue range: 20-35
            'fuchsia': [(145, 50, 50), (175, 255, 255)], # Hue range: 145-175
            'orange': [(11, 50, 50), (25, 255, 255)] # Hue range: 11-25
        }
    
    mask = np.zeros(image.shape[:2],dtype=np.uint8)

    for color_range in color_ranges.values():
        lower_color = np.array(color_range[0])
        upper_color = np.array(color_range[1])
        color_mask = cv2.inRange(hsv_image, lower_color, upper_color)
        mask = cv2.bitwise_or(mask, color_mask)

    result = cv2.bitwise_and(image, image, mask=mask)
    gray_image = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray_image, 1, 255)  # Assuming black pixels have a value of 0

    # Find the non-black points using the mask
    non_black_points = cv2.findNonZero(mask)
    points_2D = []
    
    if non_black_points is not None:
        # Iterate through the non-black points and append [x, y] to points_2D
        for point in non_black_points:
            x, y = point[0]
            points_2D.append([x + int(bboxes[0]['xmin']), y + int(bboxes[0]['ymin'])])

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

    # point_cloud = o3d.geometry.PointCloud()
    # point_cloud.points = o3d.utility.Vector3dVector(data_world)
    # o3d.io.write_point_cloud("/home/vegas/ros_ws/src/robotic_project/vision/output.pcd", point_cloud)
    # print("done")
    
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
    msg = rospy.wait_for_message("/ur5/zed_node/left_raw/image_raw_color", Image)
    detection(msg)

    s = rospy.Service('obtain_brick_pose', ObtainBrickPose, handle_obtain_brick_pose)

    print("the vision is ready to deliver the block position.")
    rospy.spin()
    
def three_points_selection(points):
    points_array=np.array(points)
    selected_points = points_array[abs(points_array[:,2] - brick_high_detection) <= brick_error]

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

    #if abs(min_y_final_point[0] - min_x_final_point[0]) <= 0.002:
    #    min_y_final_point = min_x_final_point
    #elif abs(max_y_final_point[0] - min_x_final_point[0]) <= 0.002:
    #    max_y_final_point = min_x_final_point
    
    final_points = np.array([min_y_final_point, min_x_final_point, max_y_final_point])
    return final_points

def alpha_calculation(alpha1, alpha2, distance1, distance2):
    if (alpha1==-1 and alpha2==-1):
        print("TANGENT ERROR")
        return 0
    elif (alpha2==-1 or (alpha1!=-1 and alpha1<alpha2)):
        result = alpha1
        if abs(distance1 - brick_short_side) <= lenght_error:
            return result
        else:
            return result - PI/2
    else:
        result = - alpha2
        if abs(distance2 - brick_short_side) <= lenght_error:
            return result
        else:
            return result + PI/2

def brick_center_detection(point):
    # to delete when you'll get the right AI
    global brick_long_side
    brick_long_side = 0.12

    z = brick_high_detection
    x = point[0] + ( brick_long_side*math.cos(final_alpha) + brick_short_side*math.sin(abs(final_alpha)) )/2
    y = point[1] - np.sign(final_alpha) * ( brick_short_side*math.cos(final_alpha) - brick_long_side*math.sin(abs(final_alpha)) ) / 2

    global brick_center_point
    brick_center_point = np.array([x, y, z])

def pose_detection(points):
    three_points = three_points_selection(points)
    min_y = three_points[0]
    min_x = three_points[1]
    max_y = three_points[2]

    distance1 = math.dist(min_y,min_x)
    distance2 = math.dist(min_x,max_y)

    if(distance1>minimum_distance):
        alpha1 = abs(math.atan2(min_y[0]-min_x[0],min_x[1]-min_y[1]))
    else:
        alpha1 = -1

    if(distance2>minimum_distance):
        alpha2 = abs(math.atan2(max_y[0]-min_x[0],max_y[1]-min_x[1]))
    else:
        alpha2 = -1

    print("  ")
    print("alpha1 ")
    print(alpha1)
    print("distance1 ")
    print(distance1)
    print("alpha2 ")
    print(alpha2)
    print("distance2 ")
    print(distance2)

    global final_alpha
    final_alpha = alpha_calculation(alpha1, alpha2, distance1, distance2)
    print(final_alpha)
    brick_center_detection(min_x)

if __name__ ==  '__main__':
    obtain_brick_pose_server()