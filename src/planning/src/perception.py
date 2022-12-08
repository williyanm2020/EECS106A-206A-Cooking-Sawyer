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

NUM_FOOD = 5
NUM_COLOR = 4
ARTAG_WIDTH = 0.165
BREAD_R = 0.05
MEAT_R = 0.05
BERRY_R = 0.05
LETTUCE_R = 0.05
CUP_D = 0.08

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
	def get_food_location(self,ar_tags_coor):
		if self.img is None:
			print("no img")
			return None
		ar_tag_coor,ar_tag_L,ar_tag_R,ar_tag_U,ar_tag_D = ar_tags_coor
		frame = copy.deepcopy(self.img)
		# frame = imutils.resize(frame, width=640)
		cv2.imwrite("colors.jpg",frame)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		#---- get food img coordinate----#
		#---- img center/food radius order is: 		bread -> meat -> berry -> lettuce
		#---- color mask order is: 		orange -> brown -> yellow -> green
		centers = []
		radius = [BREAD_R,MEAT_R,BERRY_R,LETTUCE_R]
		colorLower = [(0,70,200),(0,30,100),(20,50,190),(70,120,90)]			#TODO: need tunning
		colorUpper = [(20,120,230),(20,60,130),(30,80,210),(100,160,130)]		#TODO: need tunningg
		# colorLower = [(0,75,190),(0,20,80),(20,50,200),(70,140,120)]			#TODO: need tunning
		# colorUpper = [(30,100,220),(30,50,140),(50,70,240),(100,160,140)]		#TODO: need tunning
		valid_color = True
		for i in range(NUM_COLOR):
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
			else:
				# cntsSorted = sorted(cnts, key=cv2.contourArea, reverse=True)
				# cntsSorted = imutils.grab_contours(cnts)
				c = max(cntsSorted, key=cv2.contourArea)
				c = c.reshape(-1,2)
				center = np.average(c,axis=0)
				centers.append(center)
				print("contour found, center is ",center)
		# print("all food centers:",centers)
		#---- get food respect to robot coordinate----#
		for c in centers:
			cv2.circle(blurred,(int(c[0]),int(c[1])),5,(0,0,255),-1)
		cv2.imshow("final",blurred)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		for i in range(NUM_FOOD):
			self.food_coord[i] = ar_tag_coor
		if not valid_color:
			print("not valid color")
			return
		min_x,min_y = np.argmin(centers,axis=0)
		max_x,max_y = np.argmax(centers,axis=0)
		# old algo for getting coordinate
		# self.food_coord[min_x] = ar_tag_coor + np.array([0,-(radius[min_x]+ARTAG_WIDTH/2),0])	#left
		# self.food_coord[max_x] = ar_tag_coor + np.array([0,(radius[max_x]+ARTAG_WIDTH/2),0])	#right
		# self.food_coord[min_y] = ar_tag_coor + np.array([-(radius[min_y]+ARTAG_WIDTH/2),0,0])	#up
		# self.food_coord[max_y] = ar_tag_coor + np.array([(radius[max_y]+ARTAG_WIDTH/2),0,0])	#down
		# new method for getting coordinate
		self.food_coord[min_x] = ar_tag_L	#left
		self.food_coord[max_x] = ar_tag_R	#right
		self.food_coord[min_y] = ar_tag_U	#up
		self.food_coord[max_y] = ar_tag_D	#down
		self.food_coord[4] = self.food_coord[0]

	def hsvmaskTest(self):
		TEST_INTERVAL = 1000 #ms
		if self.img is None:
			print("no img")
			return None
		frame = copy.deepcopy(self.img)
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		cv2.imshow("frame",frame)
		cv2.moveWindow("frame", 100, 100)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		for i in range(1):
			print("-------------Testing hue-------------")
			cv2.imshow("original", hsv)
			cv2.moveWindow("original", 100, 100)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
			for j in range(25):
				low = j*10
				high = 255 if j==24 else (j+1)*10
				mask = cv2.inRange(hsv, (low,0,0), (high,255,255))
				cv2.imshow(str(low),mask)
				cv2.moveWindow(str(low), 100, 100)
				cv2.waitKey(TEST_INTERVAL)
				cv2.destroyAllWindows()
			print("----------Testing saturation---------")
			cv2.imshow("original", hsv)
			cv2.moveWindow("original", 100, 100)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
			for j in range(25):
				low = j*10
				high = 255 if j==24 else (j+1)*10
				mask = cv2.inRange(hsv, (0,low,0), (255,high,255))
				cv2.imshow(str(low),mask)
				cv2.moveWindow(str(low), 100, 100)
				cv2.waitKey(TEST_INTERVAL)
				cv2.destroyAllWindows()
			print("------------Testing value------------")
			cv2.imshow("original", hsv)
			cv2.moveWindow("original", 100, 100)
			cv2.waitKey(0)
			cv2.destroyAllWindows()
			for j in range(25):
				low = j*10
				high = 255 if j==24 else (j+1)*10
				mask = cv2.inRange(hsv, (0,0,low), (255,255,high))
				cv2.imshow(str(low),mask)
				cv2.moveWindow(str(low), 100, 100)
				cv2.waitKey(TEST_INTERVAL)
				cv2.destroyAllWindows()