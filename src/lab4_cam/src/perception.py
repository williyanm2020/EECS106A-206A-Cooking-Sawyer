from collections import deque
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import rospy
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2
import imutils
import copy
import math
import message_filters

NUM_FOOD = 5
NUM_COLOR = 4
ARTAG_WIDTH = 5
BREAD_R = 1
MEAT_R = 1
TOMATO_R = 1
LETTUCE_R = 1

class perception:
	def __init__(self):
		self.height = 480
		self.width = 640
		self.img = None
		self.centers = []
		self.obj_pos_cam = []
		self.food_coord = np.zeros((NUM_FOOD,3))
		rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
		# img_sub = message_filters.Subscriber("camera/color/image_raw", Image)
		# points_sub = message_filters.Subscriber("camera/depth/points", PointCloud2)
		# ts = message_filters.TimeSynchronizer([img_sub,points_sub], 10)
		# ts.registerCallback(self.combined_callback)
		return
	def img_callback(self,data):
		bridge = CvBridge()
		self.img = bridge.imgmsg_to_cv2(data, "bgr8")

	## <-- get the obj location in respect to robot (ar_tag_coor should be np array) --> ##
	def get_food_location(self,ar_tag_coor):
		if self.img is None:
			# print("no img")
			return None
		frame = copy.deepcopy(self.img)
		# frame = imutils.resize(frame, width=640)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		#---- get food img coordinate----#
		#---- img center/food radius order is: 		lower bread -> meat -> tomato -> lettuce -> upper bread
		#---- color mask order is: 		yellow -> brown -> red -> green
		centers = []
		radius = [BREAD_R,MEAT_R,TOMATO_R,LETTUCE_R,BREAD_R]
		colorLower = [(10,100,20),(20,100,100),(155,23,0),(35,25,25)]			#TODO: need tunning
		colorUpper = [(25,255,255),(30,255,255),(180,255,255),(70,255,255)]		#TODO: need tunning
		for i in range(NUM_COLOR):
			mask = cv2.inRange(hsv, colorLower[i], colorUpper[i])
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cntsSorted = sorted(cnts, key=cv2.contourArea, reverse=True)
			cntsSorted = imutils.grab_contours(cntsSorted)
			if len(cntsSorted)>0:
				c = max(cntsSorted, key=cv2.contourArea)
				c = c.reshape(-1,2)
				center = np.average(c,axis=0)
				centers.append(center)
				if i==1:
					c = cntsSorted[1]
					c = c.reshape(-1,2)
					last_center = np.average(c,axis=0)
			else:
				print("no contours found for color ",i,", change color mask!")
		centers.append(last_center)
		#---- get food respect to robot coordinate----#
		min_x,min_y = np.argmin(centers,axis=0)
		max_x,max_y = np.argmax(centers,axis=0)
		last_idx = np.sum(np.arange(NUM_FOOD)) - min_x - min_y - max_x  -max_y
		self.food_coord[min_x] = ar_tag_coor + np.array([0,-(radius[min_x]+ARTAG_WIDTH/2),0])	#left
		self.food_coord[max_x] = ar_tag_coor + np.array([0,(radius[max_x]+ARTAG_WIDTH/2),0])	#right
		self.food_coord[min_y] = ar_tag_coor + np.array([-(radius[min_y]+ARTAG_WIDTH/2),0,0])	#up
		self.food_coord[max_y] = ar_tag_coor + np.array([(radius[max_y]+ARTAG_WIDTH/2),0,0])	#down
		self.food_coord[last_idx] = ar_tag_coor

	## <-- time syncronized callback for rgb img and point cloud data --> ##
	def combined_callback(self,data1,data2):
		bridge = CvBridge()
		self.img = bridge.imgmsg_to_cv2(data1, "bgr8")
		self.centers = self.get_obj_location()
		if len(self.centers) == 0:
			self.ball_pos_cam = None
		else:
			for center in self.centers:
			    index = center[1]*self.width + center[0]
				cur_pos = list(point_cloud2.read_points(data2,field_names=("x","y","z"),skip_nans=False))[index]
				self.obj_pos_cam.append(cur_pos)
	
	## <-- get the obj location in a 2D image --> ##
	def get_obj_location(self):
		if self.img is None:
			# print("no img")
			return None
		frame = copy.deepcopy(self.img)
		# blur and color mask for orange
		num_obj = 1
		colorLower = [(10, 100, 20)]
		colorUpper = [(25, 255, 255)]
		frame = imutils.resize(frame, width=640)
		centers = []
		for i in range(num_obj):
			blurred = cv2.GaussianBlur(frame, (11, 11), 0)
			hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
			mask = cv2.inRange(hsv, colorLower[i], colorUpper[i])
			mask = cv2.erode(mask, None, iterations=2)
			mask = cv2.dilate(mask, None, iterations=2)
			# find contours in the mask and initialize the current (x, y) center of the object
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cnts = imutils.grab_contours(cnts)
			if len(cnts)>0:
				c = max(cnts, key=cv2.contourArea)
				c = c.reshape(-1,2)
				center = np.average(c,axis=0)
				# ((x, y), radius) = cv2.minEnclosingCircle(c)
				# M = cv2.moments(c)
				# center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				# cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
				centers.append(center)
		return centers

