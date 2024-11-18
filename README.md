# Camera Fix Controller

A ROS package for marker detection, tracking, and alignment using a robot's camera. This package enables the robot to gather marker information, navigate to them, and visually highlight detected markers on the camera feed.

---

## Table of Contents

1. [Features](#features)  
2. [Dependencies](#dependencies)  
3. [Installation](#installation)  
4. [Usage](#usage)  
5. [Topics](#topics)  
6. [Code Overview](#code-overview)  
7. [Author](#author)  
8. [License](#license)  

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
