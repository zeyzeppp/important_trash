#!/usr/bin/env python
#-*- coding: UTF-8 -*-

import rospy 
from ros_opencv.msg import ros_red

rospy.init_node("subscriber", anonymous = True)

def function(data):

    rospy.loginfo("[INFO].. center is calculated: (%.2f)", data.center)


rospy.Subscriber("color_detection", ros_red, function)
rospy.spin()