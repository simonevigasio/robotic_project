#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from robotic_project.srv import ObtainBrickPose, ObtainBrickPoseResponse

import numpy as np
import torch
import tf
import os

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2

from pathlib import Path
from matplotlib import pyplot as plt
from torchvision.transforms import functional as F

from sklearn.cluster import KMeans
from collections import Counter

import math

import warnings
warnings.filterwarnings("ignore")

bricks_informations = []

table_high = 0.88
brick_error = 0.001
value_error = 0.001

minimum_distance = 0.005
lenght_error = 0.005

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

color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255)], # Hue range: 0-10
            'green': [(36, 50, 50), (70, 255, 255)], # Hue range: 36-70
            'blue': [(90, 50, 50), (130, 255, 255)], # Hue range: 90-130
            'yellow': [(20, 50, 50), (35, 255, 255)], # Hue range: 20-35
            'fuchsia': [(145, 50, 50), (175, 255, 255)], # Hue range: 145-175
            'orange': [(11, 50, 50), (25, 255, 255)] # Hue range: 11-25
        }

def handle_obtain_bricks_informations(req):
    # to compute with the model
    # bricks_informations -> list of name, x, y, z, alpha
    global bricks_informations
    
    poses = []
    name = []
    for brick in bricks_informations:
        name.append(brick[0])
        pose = Pose()
        pose.position.x = brick[1]
        pose.position.y = brick[2]
        pose.position.z = brick[3]
        pose.orientation.z = brick[4]
        poses.append(pose)
    
    resp = ObtainBrickPoseResponse()
    resp.p = poses
    resp.name = name
    resp.length = len(bricks_informations)

    print('Returning brick location')
    return resp

def get_brick_value(name):
    value = np.array(brick_value[name])
    return value[0], value[1], value[2]

def correct_model_error_value(color):
    if color == 'red': return 0.03
    if color == 'green': return 0.06
    if color == 'blue': return 0.09
    if color == 'yellow': return 0.12 

def detect_color_block(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    image_hsv = image_hsv.reshape(image_hsv.shape[0]*image_hsv.shape[1], 3)
    clf = KMeans(n_clusters = 5)
    clf.fit_predict(image_hsv)
    center_colors = clf.cluster_centers_

    for color, (lower, upper) in color_ranges.items():
        if lower[0] <= center_colors[0][0] <= upper[0]:
            return color

def select_block_points_trough_mask(image, tuple):
    mask = np.zeros(image.shape[:2],dtype=np.uint8)
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    for color_range in color_ranges.values():
        lower_color = np.array(color_range[0])
        upper_color = np.array(color_range[1])
        color_mask = cv2.inRange(image_hsv, lower_color, upper_color)
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
            points_2D.append([x + int(tuple[2]), y + int(tuple[3])])
        
    return points_2D

def object_detection(image_msg: Image, point_cloud2_msg: PointCloud2, model) -> None:

    # convert received image (bgr8 format) to a cv2 image
    img = CvBridge().imgmsg_to_cv2(image_msg, "bgr8")

    # loading yolov5 model
    result = model(img)

    brick_list= []
    bboxes = result.pandas().xyxy[0].to_dict(orient="records")
    for bbox in bboxes:
        name = bbox['name']
        brick_short_side, brick_long_side, brick_high = get_brick_value(name)

        conf = bbox['confidence']
        x1 = int(bbox['xmin'])
        y1 = int(bbox['ymin'])
        x2 = int(bbox['xmax'])
        y2 = int(bbox['ymax'])

        brick_list.append((name, conf, x1, y1, x2, y2, brick_short_side, brick_long_side, brick_high))

    # iteration for each brick
    for tuple in brick_list:
        # y1 = tuple[3]
        # y2 = tuple[5]
        # x1 = tuple[2]
        # x2 = tuple[4]

        # cropping image box
        sliceBox = slice(tuple[3], tuple[5]) , slice(tuple[2],  tuple[4])
        image = img[sliceBox]

        # correct the long side of brick, using color detection
        color = detect_color_block(image)
        brick_long_side = correct_model_error_value(color)

        # filtering background
        points_2D = []
        points_2D = select_block_points_trough_mask(image, tuple)
        
        # from a list of tuples to a list of lists
        zed_points = []
        for point in list(points_2D):
            zed_points.append([int(coordinate) for coordinate in point])

        # transforming 2D points in 3D points (of the boundary box)
        points_3d = point_cloud2.read_points(point_cloud2_msg, field_names=['x','y','z'], skip_nans=False, uvs=zed_points)

        # from zed frame to world frame
        rotational_matrix = np.array([[ 0.     , -0.49948,  0.86632],
                                      [-1.     ,  0.     ,  0.     ],
                                      [-0.     , -0.86632, -0.49948]])
        
        # zed position from world frame
        pos_zed = np.array([-0.4 ,  0.59,  1.4 ])

        # selection of informations from point cloud
        zed_points = []
        for point in points_3d:
            zed_points.append(point[:3])

        # trasforming each point in world frame
        data_world = []
        for point in zed_points:
            point = rotational_matrix.dot(point) + pos_zed
            point = np.array(point)
            data_world.append(point)
            
        min_y, min_x, max_y = three_points_selection(data_world)

        alpha = brick_pose_detection(min_y, min_x, max_y, tuple[6], tuple[7],tuple[8])
        x, y, z = brick_center_detection(min_x, tuple[6], tuple[7],tuple[8], alpha)
        ros_preprocessing_data(tuple[0], x, y, z, alpha)

def ros_preprocessing_data(name, x, y, z, alpha):
    global bricks_informations
    bricks_informations.append([name, x, y, z, alpha])
    
def three_points_selection(points):
    # select all the points at a certain HIGH value
    points_array=np.array(points)
    brick_high_detection = table_high + 0.01
    selected_points = points_array[abs(points_array[:,2] - brick_high_detection) <= brick_error]

    # get index of the three points
    min_x_index = np.argmin(selected_points[:,0])
    min_y_index = np.argmin(selected_points[:,1])
    max_y_index = np.argmax(selected_points[:,1])

    # get the value of min y and max y
    min_y_value = selected_points[min_y_index, 1]
    max_y_value = selected_points[max_y_index, 1]

    # getting the set of points around min y and around max y
    min_y_selected_points = selected_points[abs(selected_points[:,1] - min_y_value) <= value_error]
    max_y_selected_points = selected_points[abs(selected_points[:,1] - max_y_value) <= value_error]

    # get the index of of the points with lowest x value
    min_y_final_index = np.argmin(min_y_selected_points[:,0])
    max_y_final_index = np.argmin(max_y_selected_points[:,0])

    # get the actual three points value
    min_x_final_point = selected_points[min_x_index]
    min_y_final_point = min_y_selected_points[min_y_final_index]
    max_y_final_point = max_y_selected_points[max_y_final_index]

    return min_y_final_point, min_x_final_point, max_y_final_point

def brick_pose_detection(min_y, min_x, max_y, brick_short_side, brick_long_side, brick_high):
    # calculate the distances of: min_y,min_x and min_x,max_y
    distance1 = math.dist(min_y,min_x)
    distance2 = math.dist(min_x,max_y)

    # if the distance is long enough calculate the hypothetical angular value 
    if(distance1>minimum_distance):
        alpha1 = abs(math.atan2(min_y[0]-min_x[0],min_x[1]-min_y[1]))
    else:
        alpha1 = -1
    if(distance2>minimum_distance):
        alpha2 = abs(math.atan2(max_y[0]-min_x[0],max_y[1]-min_x[1]))
    else:
        alpha2 = -1

    # actual angular value calculation
    if (alpha1==-1 and alpha2==-1):
        print("TANGENT ERROR")
        return 0
    elif (alpha2==-1 or (alpha1!=-1 and alpha1<alpha2)):
        if abs(distance1 - brick_short_side) <= lenght_error:
            return alpha1
        else:
            return alpha1 - PI/2
    else:
        if abs(distance2 - brick_short_side) <= lenght_error:
            return -alpha2
        else:
            return PI/2 - alpha2

def brick_center_detection(min_x, brick_short_side, brick_long_side, brick_high, alpha):
    # z is standard
    z = table_high + brick_high/2

    # using angular and sides value of the brick, calculate the center point  
    x = min_x[0] + ( brick_long_side*math.cos(alpha) + brick_short_side*math.sin(abs(alpha)) )/2
    y = min_x[1] - np.sign(alpha) * ( brick_short_side*math.cos(alpha) - brick_long_side*math.sin(abs(alpha)) ) / 2

    return x, y, z

if __name__ ==  '__main__':
    rospy.init_node('vision_node')

    #Path creation
    home_path = os.path.expanduser('~')
    path_yolo = os.path.join(home_path, "ros_ws/src/robotic_project/vision/yolov5")
    path_vision = os.path.join(home_path, "ros_ws/src/robotic_project/vision")
    path_weigths = os.path.join(path_vision, 'best.pt')

    #Model loading
    model = torch.hub.load(path_yolo, "custom", path_weigths, source='local')

    #Waiting image from zed node
    image_msg = rospy.wait_for_message("/ur5/zed_node/left_raw/image_raw_color", Image)

    ##Waiting point cloud from zed node
    point_cloud2_msg = rospy.wait_for_message("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2)

    object_detection(image_msg, point_cloud2_msg, model)

    s = rospy.Service('obtain_brick_pose', ObtainBrickPose, handle_obtain_bricks_informations)

    print("the vision is ready to deliver the block position")
    rospy.spin()