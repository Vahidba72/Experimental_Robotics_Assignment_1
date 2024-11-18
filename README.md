# Camera Fix Controller

A ROS package for marker detection, tracking, and alignment using a robot's camera. The markers are placed on a circle around the robot. This package enables the robot to gather marker information, navigate to them, and visually highlight detected markers on the camera feed. Two different codes are written. In the first python code `camera_fix_controller.py` the whole chassis of the car rotates and detects the markers which are placed around it. In the second code `camera_rotating_controller.py` only the link which cariies the camera rotates and the car chassis stays still. 

---

## Table of Contents

1. [Features](#features)  
2. [Dependencies](#dependencies)  
3. [Usage](#usage)  
4. [Topics](#topics)  
5. [Code Overview](#code-overview)  
6. [Author](#author)  


---

## Features

- Identifies markers based on their unique IDs.  
- Tracks marker positions and aligns the robot's camera center with detected markers.  
- Draws a circle around detected markers in the camera feed and publishes the processed image.  
- Provides a mechanism for the robot to rotate and gather data on multiple markers.  
- Stops rotation and alignment once all markers are processed.  

---

## Dependencies

Ensure the following software and libraries are installed:

### Required Software
- **Python 3.x**
- **ROS Noetic**

### Python Libraries
- `numpy`
- `scipy`
- `opencv-python`
- `imutils`

### Install Python dependencies:
```bash
pip install numpy scipy imutils opencv-python
```
### Usage

1. Run the camera_fix_controller node:
```bash
rosrun robot_urdf camera_fix_controller.py
```
2. Ensure the following topics are active and functioning:

- `/robot/camera1/image_raw/compressed`
- `/robot/camera1/camera_info`
- `/marker/id_number`
- `/marker/center_loc`
3. The robot will rotate to gather marker information and align its camera with the markers one by one.

### Topics
#### Published Topics
`/output/image_raw/compressed`

- Message Type: `sensor_msgs/CompressedImage`
- Description: Publishes the processed image with detected markers highlighted.

`/cmd_vel`

- Message Type: `geometry_msgs/Twist`
- Description: Publishes velocity commands to navigate the robot towards markers.

#### Subscribed Topics
`/robot/camera1/image_raw/compressed`

- Message Type: `sensor_msgs/CompressedImage`
- Description: Subscribes to the raw camera feed for marker detection.

`/robot/camera1/camera_info`

- Message Type: `sensor_msgs/CameraInfo`
- Description: Subscribes to the camera's resolution and focal properties.

`/marker/id_number`

- Message Type: `std_msgs/Int32`
- Description: Subscribes to the ID of the currently detected marker.

`/marker/center_loc`

 - Message Type: `geometry_msgs/Point`
 - Description: Subscribes to the center coordinates of the detected marker.

 ### Code Overview

The `camera_fix_controller` class is the core of this package. Below are the key methods and their functions:
`Camera_callback(self, msg)`

- Extracts the camera's center coordinates from the CameraInfo message.

`Id_callback(self, msg)`

- Updates the detected marker's ID (Id_number).

`Center_callback(self, msg)`

- Tracks the detected marker's center coordinates and stores them in a dictionary.

`Controller_callback(self, msg)`

Implements:
  - Marker detection and alignment.
  - Rotational strategy to gather all marker information.
  - Drawing circles on the processed image for visual feedback.

`main()`

- Initializes the ROS node and starts the camera_fix_controller.

### Author

**Vahid Bagherian**  
Email: [v.bagherianno@gmail.com](mailto:v.bagherianno@gmail.com)

