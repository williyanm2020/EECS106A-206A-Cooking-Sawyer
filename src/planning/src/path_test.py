#!/usr/bin/env python
"""
Path Planning Script for Lab 7
Author: Valmik Prabhu
"""
import sys
from intera_interface import Limb
import rospy
import numpy as np
import traceback
import tf2_ros as tf2
# import tf as tf
import tf.transformations as transformations
import math
import time
import copy

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from geometry_msgs.msg import Quaternion

from path_planner import PathPlanner

from intera_interface import gripper as robot_gripper

try:
    from controller import Controller
except ImportError:
    pass
    
def add_obstacles(planner):
    # # 
    # # Add the obstacle to the planning scene here
    #Obs table
    obs_const = PoseStamped()
    obs_const.header.frame_id = "base"
    obs_const.pose.position.x = 0.77
    obs_const.pose.position.y = 0
    obs_const.pose.position.z = -0.19
    obs_const.pose.orientation.x = 0
    obs_const.pose.orientation.y = 0
    obs_const.pose.orientation.z = 0
    obs_const.pose.orientation.w = 1
    planner.add_box_obstacle([1.2, 1.2, 0.05], "table", obs_const) #middle value is the length of obs



    #Obs right wall
    obs_const2 = PoseStamped()
    obs_const2.header.frame_id = "base"
    obs_const2.pose.position.x = 0.49
    obs_const2.pose.position.y = 0.65
    obs_const2.pose.position.z = 0
    obs_const2.pose.orientation.x = 0
    obs_const2.pose.orientation.y = 0
    obs_const2.pose.orientation.z = 0
    obs_const2.pose.orientation.w = 1
    planner.add_box_obstacle([1.2, 0.1, 1.2], "right wall", obs_const2) #middle value is the length of obs

    obs_const2 = PoseStamped()
    obs_const2.header.frame_id = "base"
    obs_const2.pose.position.x = 0.6
    obs_const2.pose.position.y = -0.8
    obs_const2.pose.position.z = 0
    obs_const2.pose.orientation.x = 0
    obs_const2.pose.orientation.y = 0
    obs_const2.pose.orientation.z = 0
    obs_const2.pose.orientation.w = 1
    planner.add_box_obstacle([1.2, 0.1, 1.2], "left wall", obs_const2) #middle value is the length of obs

    obs_const2 = PoseStamped()
    obs_const2.header.frame_id = "base"
    obs_const2.pose.position.x = 1.01
    obs_const2.pose.position.y = 0
    obs_const2.pose.position.z = 0
    obs_const2.pose.orientation.x = 0
    obs_const2.pose.orientation.y = 0
    obs_const2.pose.orientation.z = 0
    obs_const2.pose.orientation.w = 1
    planner.add_box_obstacle([0.1, 1.5, 1.2], "front", obs_const2) #middle value is the length of obs

    # obs_const2 = PoseStamped()
    # obs_const2.header.frame_id = "base"
    # obs_const2.pose.position.x = -0.25
    # obs_const2.pose.position.y = 0
    # obs_const2.pose.position.z = 0
    # obs_const2.pose.orientation.x = 0
    # obs_const2.pose.orientation.y = 0
    # obs_const2.pose.orientation.z = 0
    # obs_const2.pose.orientation.w = 1
    # planner.add_box_obstacle([0.1, 1.5, 1.2], "back", obs_const2) #middle value is the length of obs

    obs_const2 = PoseStamped()
    obs_const2.header.frame_id = "base"
    obs_const2.pose.position.x = 0.52
    obs_const2.pose.position.y = 0
    obs_const2.pose.position.z = 0.80
    obs_const2.pose.orientation.x = 0
    obs_const2.pose.orientation.y = 0
    obs_const2.pose.orientation.z = 0
    obs_const2.pose.orientation.w = 1
    planner.add_box_obstacle([1.2, 1.2, 0.1], "top", obs_const2) #middle value is the length of obs

def get_current_position():
    # target = "base"
    # source = "right_hand"
    # # tl = tf.TransformListener()
    # try:
    #     #pos, quat = tl.lookupTransform(target, source, rospy.Time(0))
    #     tfBuffer = tf2.Buffer(rospy.Duration(10.0))
    #     tfListener = tf2.TransformListener(tfBuffer)
    #     pos = tfBuffer.lookup_transform(target, source, rospy.Time(0))
    #     print(pos)
    # # tfBuffer = tf2_ros.Buffer()
    # # tfListener = tf2_ros.TransformListener(tfBuffer)
    # # try:
    #     # trans = tfBuffer.lookup_transform(target, source, rospy.Time())
    #     # print(trans)
    #     # translation = trans.transform.translation
    # except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
    #     print(e)
    #     return None
    # return pos
    target = "base"
    #source = "right_hand"
    source = "right_gripper_tip"

    r = rospy.Rate(10) # 10hz

    tfBuffer = tf2.Buffer()
    tfListener = tf2.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
      try:
        trans = tfBuffer.lookup_transform(target, source, rospy.Time())
        print(f"Position vector: {trans.transform.translation}")
        print(f"Quaternion: {trans.transform.rotation}")
        print()
        break
      except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException) as e:
        print(e) 
      # Use our rate object to sleep until it is time to publish again
      r.sleep()
    
    q = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
    q /= np.linalg.norm(q)
    g = transformations.quaternion_matrix(q)
    g[0,3] = trans.transform.translation.x
    g[1,3] = trans.transform.translation.y
    g[2,3] = trans.transform.translation.z
    p = [g[0,3],g[1,3],g[2,3]]
    #print("found position",p)
    return p


    

def distance_between(point1, point2):
    first = (point1[0] - point2[0]) ** 2
    second = (point1[1] - point2[1]) ** 2
    third = (point1[2] - point2[2]) ** 2
    return math.sqrt(first + second + third)

def planning(food_coord,prep_artag_loc):
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    # Calibrate the gripper (other commands won't work unless you do this first)
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # Close the right gripper
    print('Closing...')
    right_gripper.close()
    rospy.sleep(1.0)

    # Open the right gripper
    print('Opening...')
    right_gripper.open()
    rospy.sleep(1.0)
    print('Done!')
    pick_height = 0.1
    food_height = 0.01
    prep_loc = copy.deepcopy(prep_artag_loc)
    prep_loc[2] += food_height
    prep_loc_up = copy.deepcopy(prep_artag_loc)
    prep_loc_up[2] += pick_height
    food_coord_up = copy.deepcopy(food_coord)
    food_coord_up[:,2] += pick_height

    print("chech planning loc: ",prep_loc,prep_loc_up,food_coord,food_coord_up)
    for i in range(5):
        moveto(food_coord_up[i])
        moveto(food_coord[i])
        # gripper on
        right_gripper.close()
        rospy.sleep(1.0)
        moveto(food_coord_up[i])
        moveto(prep_loc_up)
        moveto(prep_loc)
        # gripper off
        right_gripper.open()
        rospy.sleep(1.0)
        moveto(prep_loc_up)
        prep_loc[2] += food_height

def moveto(loc, close=False):
    """
    Main Script
    """

    

    # Make sure that you've looked at and understand path_planner.py before starting
    planner = PathPlanner("right_arm")

    Kp = 0.2 * np.array([0.4, 2, 1.7, 1.5, 2, 2, 3])
    Kd = 0.01 * np.array([2, 1, 2, 0.5, 0.8, 0.8, 0.8])
    Ki = 0.01 * np.array([1.4, 1.4, 1.4, 1, 0.6, 0.6, 0.6])
    Kw = np.array([0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])


    add_obstacles(planner)

    length_threshold = 0.310248/20 #real is 13
    length_threshold2 = 0.275/10 #real is 6

    # #Create a path constraint for the arm
    # #UNCOMMENT FOR THE ORIENTATION CONSTRAINTS PART
    orien_const = OrientationConstraint()
    orien_const.link_name = "right_gripper"
    orien_const.header.frame_id = "base"
    orien_const.orientation.y = 1.0
    orien_const.absolute_x_axis_tolerance = 0.1
    orien_const.absolute_y_axis_tolerance = 0.1
    orien_const.absolute_z_axis_tolerance = 0.1
    #orien_const.parameterization = 1
    orien_const.weight = 1.0
    constraints = []
    # constraints.append(orien_const)
    

    cont = Controller(Kp, Ki, Kd, Kw, Limb("right"))
    cont_imp = True
    right_gripper = robot_gripper.Gripper('right_gripper')

    while not rospy.is_shutdown():
        repeat_execution = True
        # user_input_location_x = float(input("Position x: "))
        # user_input_location_y = float(input("Position y: "))
        # user_input_location_z = float(input("Position z: "))
        user_input_location_x,user_input_location_y,user_input_location_z = loc

        curr_pos = get_current_position()
        print("start moving to location: ", loc)
        print(f"Current position: {curr_pos}")
        print(f"Current distance: {distance_between(curr_pos, [user_input_location_x, user_input_location_y, user_input_location_z])}")

        print("Now, let's move to the new location")

        curr_dist = distance_between(curr_pos, [user_input_location_x, user_input_location_y, user_input_location_z])
        while repeat_execution:
            try:
                # x, y, z = 0.8, 0.05, 0.07
                
                # move_to([0.801, 0.348, 0.157])
                # print("Now, let's move to the new location")
                # move_to([0.816, -0.295, 0.275])

                goal_1 = PoseStamped()
                goal_1.header.frame_id = "base"

                #x, y, and z position
                goal_1.pose.position.x = user_input_location_x
                goal_1.pose.position.y = user_input_location_y
                goal_1.pose.position.z = user_input_location_z

                #Orientation as a quaternion
                # goal_1.pose.orientation.x = 0.0
                # goal_1.pose.orientation.y = -1.0
                # goal_1.pose.orientation.z = 0.0
                # goal_1.pose.orientation.w = 0.0
                goal_1.pose.orientation.x = 0.0
                goal_1.pose.orientation.y = 1.0
                goal_1.pose.orientation.z = 0.0
                goal_1.pose.orientation.w = 0.0

                plan_length = 100000
                iters = 0
                # Might have to edit this . . . 
                plans = {}
                # while curr_dist / plan_length < length_threshold:
                #     iters += 1
                #     print(f"Iteration: {iters}. planning new path....")
                #     plan = planner.plan_to_pose(goal_1, constraints)

                #     plan_length = len(plan[1].joint_trajectory.points)
                #     plans[plan_length] = plan
                #     print(f"Plan is the following length: {len(plan[1].joint_trajectory.points)}")
                #     if iters == 5:
                #         break
                #     time.sleep(1)

                while iters < 20:
                    iters += 1
                    print(f"Iteration: {iters}. planning new path....")
                    plan = planner.plan_to_pose(goal_1, constraints)

                    plan_length = len(plan[1].joint_trajectory.points)
                    plans[plan_length] = plan
                    print(f"Plan is the following length: {len(plan[1].joint_trajectory.points)}")
                    time.sleep(0.05)
                min_length = min(plans.keys())
                plan = plans[min_length]                         

                # input("Press <Enter> to move the right arm to goal pose 1: ")
                # if cont_imp:
                #     if not cont.execute_plan(plan[1]): 
                #         raise Exception("Execution failed")
                # else:
                #     if not planner.execute_plan(plan[1]): 
                #         raise Exception("Execution failed")

            
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ, else 'n': ")
            
                if user_input == 'y':
                    if cont_imp:
                        cont.execute_plan(plan[1], log=False)
                    else:
                        planner.execute_plan(plan[1])
                    repeat_execution = False
                    
                    # # User input if want to open or closer gripper
                    # user_input_gripper = input("Enter 'c' if close gripper, 'o' if open gripper, else 'n': ")

                    # if user_input_gripper == 'c':
                    #     right_gripper.close()
                    # elif user_input_gripper == 'o':
                    #     right_gripper.open()
                    # else:
                    #     break
                    # open = True
                    # if user_input_gripper == 'y':
                    #     group.execute(plan[1])
                    #     if open:
                    #         right_gripper.close()
                    #         open = False
                    #     else:
                    #         right_gripper.open()
                    #     return

                    # if close:
                    #     right_gripper.close()
                    # else:
                    #     right_gripper.open()

                elif user_input == 'n':
                    repeat_execution = False
                else:
                    break

            except Exception as e:
                print(e)
                traceback.print_exc()
            else:
                break
        break

    
if __name__ == '__main__':
    rospy.init_node('moveit_node')
    loc = [0.851, 0.103, -0.142]
    loc2 = [0.766, -0.195, -0.130]
    loc3 = [0.801, 0.348, 0.157]
    loc4 = [0.816, -0.295, 0.275]
    moveto(loc, close=True)
    moveto(loc2)
    moveto(loc3, close=True)
    moveto(loc4)
    #planning()