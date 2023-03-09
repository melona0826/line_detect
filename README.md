# line_detect

The package of line detector.

# Enviornment

Ubuntu 18.04 + ROS melodic

# How to run detect Node

**roslaunch line_detect detect.launch**

# Node description

* **/yellow_detect/yellow_line_pos : [geometry_msgs/Pose2D] 2D Point (x,y) that on a fit yellow line and theta (radian) that degree of fit yellow line.**

* **/yellow_detect/yellow_detect_img : Image with fit line with yellow line (The red line that final line), straight line (blue line)**

* **/yellow_detect/yellow_img : Image that filtered only yellow line**

* **/white_detect/yellow_line_pos : [geometry_msgs/Pose2D] 2D Point (x,y) that on a fit white line and theta (radian) that degree of fit white line.**

* **/white_detect/white_detect_img : Image with fit line with white line (The red line that final line), straight line (blue line)**

* **/white_detect/white_img : Image that filtered only white line**
