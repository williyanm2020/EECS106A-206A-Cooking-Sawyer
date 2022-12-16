# EECS106A-206A-Cooking-Sawyer

Goal: To make Sawyer to pick up burget ingredients from the ingredients area and assemble them in the preparation area. 

The cooking Sawyer is able to identify food ingredients at random locations on the table using computer vision (colour thresholding) and follow a desired recipe to assemble a customized burger using motion planning.

Project Link: 
https://sites.google.com/view/cookingsawyer/home

Setup Instructions:
1. Download ar_track_alvar package. First go to folder "src", then:
  git clone https://github.com/machinekoder/ar_track_alvar.git -b noetic-devel
2. Place camera calibration file in ros (do it in folder "src"):
  mv camera_info ˜/.ros
3. Build workspace: catkin_make
4. Reset Sawyer position:
  roslaunch intera_examples sawyer_tuck.launch
5. Launch camera: 
  roslaunch lab4_cam run_cam.launch
6. Launch ar_track: 
  roslaunch lab4_cam ar_track.launch
7. Put the ARtag to the food prep area on the table
8. Launch moveit:
  roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true
9. Visualize: 
  rosrun rviz rviz
10. Run this file to execute: 
  rosrun planning coordinate.py
11. Put food on ar_tags before color segmentation
