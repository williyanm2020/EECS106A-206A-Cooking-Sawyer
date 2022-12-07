#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf.transformations as transformations
import tf2_ros
import sys
import numpy as np

from geometry_msgs.msg import Twist
from perception import perception
import path_test

# Setup Instructions
# 1. Build workspace: catkin_make
# 2. Launch camera: 
#   roslaunch lab4_cam run_cam.launch
# 3. Launch ar_track: 
#   roslaunch lab4_cam ar_track.launch
# 4. Put the ARtag to the food prep area on the table
# 5. Launch moveit:
#   roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
# 5. Set up your environment, make a shortcut (symbolic link) to the Sawyer environment script: 
#   ln -s /opt/ros/eecsbot_ws/intera.sh ~/CookingSawyer/
# 6. ssh into one of the Sawyer robots: 
#   ./intera.sh [name-of-robot].local
# 7. Visualize: 
#   export ROS_MASTER_URI=http://[name-of-robot].local:11311
#   rosrun rviz rviz
# 8. Move the sawyer arm in zero gravity mode, pointing the gripper down to the ARtag
# 9. Run this file to set up coordinate frames and transformation: 
#   python3 coordinate.py



class Coordinate(object):
  def __init__(self):
    """
    setup the coordinate system transform between the camera and sawyer base frame
    """

    self.sawyer_base = "base" # frame a
    self.sawyer_gripper = "right_hand" # frame b
    self.prep_artag = "ar_marker_2" # frame c
    self.cam_base = "usb_cam" # frame d
    self.food_artag = "ar_marker_13" # frame e
    self.base_artag = "ar_marker_0"  # frame f
    self.food_artag_L = "ar_marker_5"  # frame g
    self.food_artag_R = "ar_marker_3"  # frame h
    self.food_artag_U = "ar_marker_6"  # frame i
    self.food_artag_D = "ar_marker_7"  # frame j
    # self.gripper_len = 0.095 # customized 3D-printed gripper
    self.gripper_len = 0.2
    # self.artag_len = 0.00825 # 16.5cm / 2
    self.artag_len = 0.03 # 6cm / 2
    self.base_radius = 0.1143 + 0.29 # added ruler
    # self.g_ab = self.tf_trans(self.sawyer_base, self.sawyer_gripper)
    # self.g_bc = np.ndarray([[0,1,0,0],[1,0,0,0],[0,0,-1,self.gripper_len],[0,0,0,1]])
    # self.g_cd = self.tf_trans(self.prep_artag, self.cam_base)
    # self.g_ad = self.g_ab @ self.g_bc @ self.g_cd
    # self.g_ed = self.tf_trans(self.food_artag, self.cam_base)

    #<--- zhiqi think this way --->#
    # old FK
    self.g_ab = self.tf_trans(self.sawyer_gripper,self.sawyer_base)
    # self.g_bc = np.array([[0,1,0,self.artag_len],[1,0,0,0],[0,0,-1,self.gripper_len],[0,0,0,1]])
    # self.g_cd = self.tf_trans(self.cam_base, self.prep_artag)
    # self.g_ad = self.g_ab @ self.g_bc @ self.g_cd
    # new FK
    self.g_af = np.array([[0,0,1,self.base_radius],[1,0,0,0],[0,1,0,0],[0,0,0,1]])
    self.g_bf = np.array([[0,0,-1,-0.044],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    self.g_fd = self.tf_trans(self.cam_base,self.base_artag)
    self.g_ad = self.g_ab @ self.g_bf @ self.g_fd
    # self.g_ad = self.g_af @ self.g_fd
    # get food artag and prep artag loc
    # g_de = self.tf_trans(self.food_artag, self.cam_base)
    g_dc = self.tf_trans(self.prep_artag, self.cam_base)
    g_dg = self.tf_trans(self.food_artag_L, self.cam_base)
    g_dh = self.tf_trans(self.food_artag_R, self.cam_base)
    g_di = self.tf_trans(self.food_artag_U, self.cam_base)
    g_dj = self.tf_trans(self.food_artag_D, self.cam_base)
    # food_artag_cam = [g_de[0,3],g_de[1,3],g_de[2,3]]
    prep_artag_cam = [g_dc[0,3],g_dc[1,3],g_dc[2,3]]
    food_artag_L_cam = [g_dg[0,3],g_dg[1,3],g_dg[2,3]]
    food_artag_R_cam = [g_dh[0,3],g_dh[1,3],g_dh[2,3]]
    food_artag_U_cam = [g_di[0,3],g_di[1,3],g_di[2,3]]
    food_artag_D_cam = [g_dj[0,3],g_dj[1,3],g_dj[2,3]]

    # self.food_artag_loc = self.coordinate_change(food_artag_cam)
    self.prep_artag_loc = self.coordinate_change(prep_artag_cam)
    self.food_artag_L_loc = self.coordinate_change(food_artag_L_cam)
    self.food_artag_R_loc = self.coordinate_change(food_artag_R_cam)
    self.food_artag_U_loc = self.coordinate_change(food_artag_U_cam)
    self.food_artag_D_loc = self.coordinate_change(food_artag_D_cam)

    print("Coordinate init done.")

  # Define the method which contains the node's main functionality
  def tf_trans(self, target_frame, source_frame):
    """
    Computes the SE3 transformation between two frames.
    
    Args:
    target_frame: string, tf_frame
    source_frame: string, tf_frame
    
    Returns:
    g - (4,4) the resulting transformation matrix (SE3)
    """

    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    r = rospy.Rate(10) # 10hz

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
      try:
        trans = tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time())
        print(f"Position vector: {trans.transform.translation}")
        print(f"Quaternion: {trans.transform.rotation}")
        print()
        break
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e) 
      # Use our rate object to sleep until it is time to publish again
      r.sleep()
    q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
    q /= np.linalg.norm(q)
    g = transformations.quaternion_matrix(q)
    g[0,3] = trans.transform.translation.x
    g[1,3] = trans.transform.translation.y
    g[2,3] = trans.transform.translation.z

    return g
  
  def coordinate_change(self, cam_point):
    """
    Change a point coordinate from camera base frame (frame d) to sawyer base frame (frame a)
    Args:
    cam_point: (3,) point in camera base frame
    
    Returns:
    sawyer_point: (3,) point in sawyer base frame
    """
    
    sawyer_point = np.dot(self.g_ad, cam_point+[1])
    assert sawyer_point[3] == 1
    sawyer_point = sawyer_point[0:3]
    return sawyer_point

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('coordinate_node', anonymous=True)

  # computer vision part
  TABLE_H = -0.19
  CUP_L = 0.095
  ALL_Z = TABLE_H + CUP_L
  coord = Coordinate()
  print("g_ad:",coord.g_ad)
  print("g_ab:",coord.g_ab)
  print("g_bf:",coord.g_bf)
  # print("g_af",coord.g_af)
  print("g_fd:",coord.g_fd)
  # print("food_artag_loc:",coord.food_artag_loc)
  print("prep_artag_loc:",coord.prep_artag_loc)
  print("food_artag_L:",coord.food_artag_L_loc,"\n food_artag_R:",coord.food_artag_R_loc,"\n food_artag_U:",coord.food_artag_U_loc,"\n food_artag_D:",coord.food_artag_D_loc)
  
  # perc = perception()
  # temp = input()
  # perc.get_food_location([coord.food_artag_loc,coord.food_artag_L_loc,coord.food_artag_R_loc,coord.food_artag_U_loc,coord.food_artag_D_loc])
  # food_coord = perc.food_coord
  # # food_coord[:,2] += (CUP_L+coord.gripper_len/2)
  # food_coord[:,2] += (CUP_L)
  # prep_loc = coord.prep_artag_loc
  # # prep_loc[2] += coord.gripper_len
  # print("food_coord result:",food_coord)
  #TODO: motion planning part fill in
  
  offset = 0.165/2 + 0.05
  # prep_loc = np.array([0.711, 0.110, -0.152+0.095])
  # food_coord = np.array([[0.761-offset,-0.272,-0.145+0.095],[0.761+offset,-0.272,-0.145+0.095],[0.761,-0.272-offset,-0.145+0.095],[0.761,-0.272+offset,-0.145+0.095]])
  prep_loc = coord.prep_artag_loc
  food_coord = np.array([coord.food_artag_L_loc,coord.food_artag_R_loc,coord.food_artag_U_loc,coord.food_artag_D_loc])
  prep_loc[2] = ALL_Z
  food_coord[:,2] = ALL_Z

  path_test.planning(food_coord,prep_loc)


  # testing for cv
  # perc = perception()
  # a = input()
  # perc.get_food_location(np.array([0,0,1]))
  # food_coord = perc.food_coord
  # print("food_coord result:",food_coord)