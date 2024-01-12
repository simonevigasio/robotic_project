
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
├── srv
│   └── ObtainBrickPose.srv
└── vision
    ├── best.pt
    ├── vision.py
    └── yolov5
```