#!/usr/bin/env python3

# Ros libraries
import roslib
import rospy
import time
import math

# Python libs
import sys

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros Messages

from cv_bridge import CvBridge, CvBridgeError


from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist , Point 
from sensor_msgs.msg import CompressedImage , CameraInfo
from std_msgs.msg import Float64


class camera_rotating_controller:

	def __init__(self):
	
		rospy.init_node('camera_rotating_controller')
		
		
		
		self.CameraCenter = Point()
		self.MarkerCenter = Point()
		self. marker_id_list = list()
		self.marker_centers_dict = {}  # Dictionary to hold center coordinates for each marker detected
		self.Id_number = 0
		self.Info_gathering_mode = True  # The flag to control when to stop the data gathering 
		self.Reached = False
		self.Current_marker = 0
		self.Current_angle = 0.0
		
				
		# Publishers
		
		self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
		
		self.camera_holder_publisher = rospy.Publisher('/my_robot4/camera_holder_position_controller/command' , Float64 , queue_size = 1)
		
		
		# Subscribers
		
		rospy.Subscriber("/robot/camera1/image_raw/compressed" , CompressedImage , self.Controller_callback , queue_size = 1)
		
		rospy.Subscriber("/robot/camera1/camera_info" , CameraInfo , self.Camera_callback , queue_size = 1)
		
		rospy.Subscriber("/marker/id_number" , Int32 , self.Id_callback , queue_size = 1)
		
		rospy.Subscriber("/marker/center_loc" , Point , self.Center_callback , queue_size = 1)
		
	# This callback is to find the center of the camera 
	def Camera_callback(self , msg ):
	
		self.CameraCenter.x = msg.height/2
		self.CameraCenter.y = msg.width/2
	
	# This callback is to find the id number for the marker	
	def Id_callback(self , msg):
	
		self.Id_number = msg.data
		#rospy.loginfo(f"Updated Id_number: {self.Id_number}")
		
	
	def Center_callback (self , msg ):
	
		self.MarkerCenter.x = msg.x
		self.MarkerCenter.y = msg.y
	
		if self.Id_number and self.Id_number not in self.marker_id_list:
		
			self.marker_centers_dict[self.Id_number] = (msg.x , msg.y)
			#rospy.loginfo(f"Updated marker centers: {self.marker_centers_dict}")
			
			
				
	
	
	def Controller_callback(self , msg):
	
		#### direct conversion to CV2 ####
		
		
		np_arr = np.fromstring(msg.data, np.uint8)
		
		image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
	
		
		vel = Float64()
		
		
		if self.Info_gathering_mode:
		
			if len(self.marker_id_list) < 7: 
				
				
				self.Current_angle += 0.07 
				vel.data = self.Current_angle

				if self.Id_number and self.Id_number not in self.marker_id_list:

					self.marker_id_list.append(self.Id_number)
					self.marker_id_list.sort()
					rospy.loginfo(f"Updated marker ids: {self.marker_id_list}")
				
				
		
			else:
				
				self.Info_gathering_mode = False
				rospy.loginfo("Finished gathering all markers. Starting drawing phase.")
				
		else:
		
			
			
			if len(self.marker_id_list)>0:
			
				self.Current_marker = self.marker_id_list[0]
				target_x = self.MarkerCenter.x
				target_y = self.MarkerCenter.y
				print('taregt is', target_x , 'center is' , self.CameraCenter.x , 'Looking for' , self.Current_marker)
				
				
				
				if abs(self.CameraCenter.x - target_x) < 20 and self.Id_number == self.Current_marker:
				
					self.Reached = True
					self.Current_angle += 0 
					vel.data = self.Current_angle 
					rospy.loginfo(f"Reached marker {self.Current_marker}")
					
					# Draw a circle around the marker position on the image
					
					cv2.circle(image_np, (int(target_x), int(target_y)), 20, (0, 255, 0), 3)
					
					Image_msg = CompressedImage()
					Image_msg.header.stamp = rospy.Time.now()
					Image_msg.format = "jpeg"
					Image_msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
					
					self.image_pub.publish(Image_msg)
					
					
				if self.Reached:
				
					self.marker_id_list.pop(0)
					self.Reached = False
					self.Current_angle += 0
					vel.data = self.Current_angle
					
					
				elif self.CameraCenter.x > target_x and self.Id_number == self.Current_marker:
					
					self.Current_angle += 0.01
					vel.data = self.Current_angle 
					print('turn left')
					
				elif self.CameraCenter.x < target_x  and self.Id_number == self.Current_marker:
				
					self.Current_angle -= 0.01
					vel.data = self.Current_angle 
					print('turn right')
				else: 
					self.Current_angle += 0.07
					vel.data = self.Current_angle
					print('keep up')
				
			else:
			
				self.Current_angle += 0
				vel.data = self.Current_angle
				print("all markers found")	
					
				
		self.camera_holder_publisher.publish(vel)    
		
		
				
		
			

def main():

	camera_rotating_controller()
	rospy.spin()
	
	
if __name__ == '__main__':

	main()		
		

