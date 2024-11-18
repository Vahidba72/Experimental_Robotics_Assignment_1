<?xml version="1.0" encoding="UTF-8"?>
<readme>
  <title>Camera Fix Controller</title>
  <description>
    This ROS package implements a camera-based marker detection and navigation system. 
    The robot rotates to gather marker information, identifies marker positions, and visually highlights them on a camera feed.
  </description>
  
  <features>
    <feature>Identifies markers using their IDs.</feature>
    <feature>Publishes marker positions and highlights them on a visual feed.</feature>
    <feature>Uses a proportional control mechanism to align the camera center with marker positions.</feature>
  </features>
  
  <dependencies>
    <dependency>Python 3.x</dependency>
    <dependency>ROS Noetic</dependency>
    <dependency>OpenCV</dependency>
    <dependency>Numpy</dependency>
    <dependency>Scipy</dependency>
    <dependency>Imutils</dependency>
  </dependencies>
  
  <usage>
    <step>
      <description>Clone this repository and navigate to the package directory.</description>
      <command>git clone &lt;repository_url&gt;</command>
    </step>
    <step>
      <description>Ensure you have all dependencies installed.</description>
      <command>pip install numpy scipy imutils opencv-python</command>
    </step>
    <step>
      <description>Build your ROS workspace.</description>
      <command>catkin_make</command>
    </step>
    <step>
      <description>Source the workspace.</description>
      <command>source devel/setup.bash</command>
    </step>
    <step>
      <description>Run the camera_fix_controller node.</description>
      <command>rosrun &lt;your_package_name&gt; camera_fix_controller.py</command>
    </step>
  </usage>
  
  <topics>
    <publisher>
      <topic>/output/image_raw/compressed</topic>
      <type>sensor_msgs/CompressedImage</type>
      <description>Publishes the processed image with highlighted markers.</description>
    </publisher>
    <publisher>
      <topic>/cmd_vel</topic>
      <type>geometry_msgs/Twist</type>
      <description>Publishes velocity commands to control robot movement.</description>
    </publisher>
    <subscriber>
      <topic>/robot/camera1/image_raw/compressed</topic>
      <type>sensor_msgs/CompressedImage</type>
      <description>Subscribes to the raw camera feed.</description>
    </subscriber>
    <subscriber>
      <topic>/robot/camera1/camera_info</topic>
      <type>sensor_msgs/CameraInfo</type>
      <description>Subscribes to camera information for resolution details.</description>
    </subscriber>
    <subscriber>
      <topic>/marker/id_number</topic>
      <type>std_msgs/Int32</type>
      <description>Subscribes to marker ID information.</description>
    </subscriber>
    <subscriber>
      <topic>/marker/center_loc</topic>
      <type>geometry_msgs/Point</type>
      <description>Subscribes to the marker center location.</description>
    </subscriber>
  </topics>
  
  <author>
    <name>Vahid Bagherian</name>
    <email>your_email@example.com</email>
  </author>
  
  <license>MIT</license>
</readme>
