
# Robotic Project
Foundament of Robotic project year 2023/2024

Authors:
1. Gabriel Fumagalli
2. Simone Vigasio
3. Stefan Gore

## Project Description

The objective of this project was to create a self-sufficient robot capable of executing pick-and-place operations. The robotic arm employed for manipulation is the Ur5, equipped with a zed camera for perception. The pick-and-place operation involves selecting various bricks and accurately positioning them in their predetermined locations. The robot possesses the capability to autonomously identify the bricks and execute the pick-and-place task. 
![alt text](https://github.com/SV00/robotic_project/blob/master/utilities/robot_image.jpeg?raw=true)

## Folder

```
.
├── CMakeLists.txt
├── include
│   └── robotic_project
├── models
├── package.xml
├── src
│   └── motion
│       ├── Eigen
│       ├── kinematics.cpp
│       └── main.cpp
├── srv
│   └── ObtainBrickPose.srv
└── vision
    ├── best.pt
    ├── vision.py
    └── yolov5
```
This project consists of two ROS nodes. One is specifically designed for object detection of the bricks on the table, and it calculates their localization and orientation. The other node is focused on robot motion and task planning.

The link between the two nodes is established through a service. The "vision.py" file functions as the server, while the "main.cpp" file acts as the client, tasked with patiently waiting until the brick detection is complete.

## Installation
The project was tested on ubuntu 20.04 with ROS Noetic. 
To make it works we use [locosim](https://github.com/mfocchi/locosim) repository. 
So, first of all clone locosim repository on your machine following the respective instructions.

Inside the ros_ws/src where you have imported the locosim package, write this command:
```
git clone https://github.com/SV00/robotic_project.git
```
Now, you have to install the vision component. Inside the ros_ws directory write:
```
cd src/robotic_project/vision
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip3 install -r requirements.txt
pip install torchvision==0.13.0
```
Finally, always inside ros_ws folder compile the project:
```
catkin_make install
. devel/setup.bash
```
