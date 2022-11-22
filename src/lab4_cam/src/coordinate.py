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

# Setup Instructions
# 1. Build workspace: catkin_make
# 2. Launch camera: 
#   roslaunch lab4_cam run_cam.launch
# 3. Launch ar_track: 
#   roslaunch lab4_cam ar_track.launch
# 4. Put the ARtag to the food prep area on the table
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
    self.table_artag = "ar_marker_0" # frame c
    self.cam_base = "usb_cam" # frame d
    self.gripper_len = 0.095
    self.g_ab = self.tf_trans(self.sawyer_base, self.sawyer_gripper)
    self.g_bc = [[0,1,0,0],[1,0,0,0],[0,0,-1,self.gripper_len],[0,0,0,1]]
    self.g_cd = self.tf_trans(self.table_artag, self.cam_base)
    self.g_ad = self.g_ab @ self.g_bc @ self.g_cd
    print("Coordinate init done.")

  # Define the method which contains the node's main functionality
  def tf_trans(target_frame, source_frame):
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
        trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
        print(f"Position vector: {trans.transform.translation}")
        print()
        print(f"Quaternion: {trans.transform.rotation}")
        break
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e) 
      # Use our rate object to sleep until it is time to publish again
      r.sleep()
    q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
    q /= np.linalg.norm(q)
    g = transformations.quaternion_matrix(q)
    g[0,3] = trans.transform.position.x
    g[1,3] = trans.transform.position.y
    g[2,3] = trans.transform.position.z

    return g
  
  def coordinate_change(cam_point):
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
  coord = Coordinate()

  rospy.spin()

