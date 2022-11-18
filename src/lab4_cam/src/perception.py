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

class perception:
	def __init__(self):
		self.height = 480
		self.width = 640
		self.img = None
		self.centers = []
		self.obj_pos_cam = []
		img_sub = message_filters.Subscriber("camera/color/image_raw", Image)
		points_sub = message_filters.Subscriber("camera/depth/points", PointCloud2)
		ts = message_filters.TimeSynchronizer([img_sub,points_sub], 10)
		ts.registerCallback(self.combined_callback)
		return
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

