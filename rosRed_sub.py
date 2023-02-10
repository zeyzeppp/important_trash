#!/usr/bin/env python
#-*- coding: UTF-8 -*-

import rospy 
from ros_colorDetection.msg import rosRed

def function(data):

    rospy.loginfo("[INFO].. center is calculated (%.2f):", data.center)

rospy.init_node("subscriber", anonymous = True)

rospy.Subscriber("color_detection", rosRed, function)
rospy.spin()