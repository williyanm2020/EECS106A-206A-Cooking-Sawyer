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

NUM_FOOD = 4
NUM_COLOR = 4
ARTAG_WIDTH = 0.0165
BREAD_R = 0.05
MEAT_R = 0.05
BERRY_R = 0.05
LETTUCE_R = 0.05

class perception():
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
			print("no img")
			return None
		frame = copy.deepcopy(self.img)
		# frame = imutils.resize(frame, width=640)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		#---- get food img coordinate----#
		#---- img center/food radius order is: 		bread -> meat -> berry -> lettuce
		#---- color mask order is: 		brown -> yellow -> purple -> green
		centers = []
		radius = [BREAD_R,MEAT_R,BERRY_R,LETTUCE_R]
		colorLower = [(5,50,0),(25,80,20),(110,75,120),(70,100,50)]			#TODO: need tunning
		colorUpper = [(25,255,255),(50,255,255),(140,180,180),(100,255,255)]		#TODO: need tunning
		valid_color = True
		for i in range(NUM_COLOR):
			# if i==2:
			# 	for j in range(25):
			# 		mask = cv2.inRange(hsv, (110,j*10,0), (140,(j+1)*10,255))
			# 		cv2.imshow(str(j),mask)
			# 		cv2.waitKey(300)
			# 		cv2.destroyAllWindows()
			# 	continue
			mask = cv2.inRange(hsv, colorLower[i], colorUpper[i])
			# mask = cv2.inRange(blurred, colorLower[i], colorUpper[i])
			cv2.imshow("original",hsv)
			cv2.waitKey(0)
			cv2.imshow("mask",mask)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
			cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			cntsSorted = imutils.grab_contours(cnts)
			# cnts = cnts[0] if imutils.is_cv2() else cnts[1] 
			# print("check1:",cntsSorted,len(cntsSorted))
			if cntsSorted is None or len(cntsSorted) == 0:
				valid_color = False
				centers.append([0,0])
				print("no contours found for color ",i,", change color mask!")
				if i==0:
					last_center = [0,0]
			else:
				# cntsSorted = sorted(cnts, key=cv2.contourArea, reverse=True)
				# cntsSorted = imutils.grab_contours(cnts)
				c = max(cntsSorted, key=cv2.contourArea)
				c = c.reshape(-1,2)
				center = np.average(c,axis=0)
				centers.append(center)
				if i==0:
					# c = cntsSorted[1]
					# c = c.reshape(-1,2)
					# last_center = np.average(c,axis=0)
					last_center = center
				print("contour found, center is ",center)
		centers.append(last_center)
		# print("all food centers:",centers)
		#---- get food respect to robot coordinate----#
		for c in centers:
			cv2.circle(blurred,(int(c[0]),int(c[1])),5,(0,0,255),-1)
		cv2.imshow("final",blurred)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		if not valid_color:
			print("not valid color")
			return []
		for i in range(NUM_FOOD):
			self.food_coord[i] = ar_tag_coor
		min_x,min_y = np.argmin(centers,axis=0)
		max_x,max_y = np.argmax(centers,axis=0)
		# last_idx = np.sum(np.arange(NUM_FOOD)) - min_x - min_y - max_x  -max_y
		self.food_coord[min_x] = ar_tag_coor + np.array([0,-(radius[min_x]+ARTAG_WIDTH/2),0])	#left
		self.food_coord[max_x] = ar_tag_coor + np.array([0,(radius[max_x]+ARTAG_WIDTH/2),0])	#right
		self.food_coord[min_y] = ar_tag_coor + np.array([-(radius[min_y]+ARTAG_WIDTH/2),0,0])	#up
		self.food_coord[max_y] = ar_tag_coor + np.array([(radius[max_y]+ARTAG_WIDTH/2),0,0])	#down
		# self.food_coord[last_idx] = ar_tag_coor

	## <-- time syncronized callback for rgb img and point cloud data --> ##
	# def combined_callback(self,data1,data2):
	# 	bridge = CvBridge()
	# 	self.img = bridge.imgmsg_to_cv2(data1, "bgr8")
	# 	self.centers = self.get_obj_location()
	# 	if len(self.centers) == 0:
	# 		self.ball_pos_cam = None
	# 	else:
	# 		for center in self.centers:
	# 		    index = center[1]*self.width + center[0]
	# 			cur_pos = list(point_cloud2.read_points(data2,field_names=("x","y","z"),skip_nans=False))[index]
	# 			self.obj_pos_cam.append(cur_pos)
	
	# ## <-- get the obj location in a 2D image --> ##
	# def get_obj_location(self):
	# 	if self.img is None:
	# 		# print("no img")
	# 		return None
	# 	frame = copy.deepcopy(self.img)
	# 	# blur and color mask for orange
	# 	num_obj = 1
	# 	colorLower = [(10, 100, 20)]
	# 	colorUpper = [(25, 255, 255)]
	# 	frame = imutils.resize(frame, width=640)
	# 	centers = []
	# 	for i in range(num_obj):
	# 		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	# 		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	# 		mask = cv2.inRange(hsv, colorLower[i], colorUpper[i])
	# 		mask = cv2.erode(mask, None, iterations=2)
	# 		mask = cv2.dilate(mask, None, iterations=2)
	# 		# find contours in the mask and initialize the current (x, y) center of the object
	# 		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	# 		cnts = imutils.grab_contours(cnts)
	# 		if len(cnts)>0:
	# 			c = max(cnts, key=cv2.contourArea)
	# 			c = c.reshape(-1,2)
	# 			center = np.average(c,axis=0)
	# 			# ((x, y), radius) = cv2.minEnclosingCircle(c)
	# 			# M = cv2.moments(c)
	# 			# center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	# 			# cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
	# 			cv2.circle(frame, center, 5, (0, 0, 255), -1)
	# 			centers.append(center)
	# 	return centers
