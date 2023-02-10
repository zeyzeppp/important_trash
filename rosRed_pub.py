#!/usr/bin/env python
#-*- coding: UTF-8 -*-


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from ros_color_detection.msg import rosRed
from cv_bridge import CvBridge, CvBridgeError



rospy.init_node("colorPublisher", anonymous= True)

bridge = CvBridge()

def image_callback(ros_image):

    print("got an image")
    global bridge

    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")

    except CvBridgeError as e:
        print(e)


    #cap = cv2.VideoCapture("resources/redColor.mp4")
    cap = cv2.VideoCapture(0)


    lowRed = np.array([161, 155, 84])
    upRed = np.array([179, 255, 255])


    while True:
        ret, frame = cap.read()
        frame = cv2.resize(frame, (540, 960))
        #frame = cv2.flip(frame, 1)
        hsv_zey = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_mask = cv2.inRange(hsv_zey, lowRed, upRed)
        red = cv2.bitwise_and(frame, frame, mask = red_mask)
        red = cv2.resize(red, (540, 960))
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        x, y, w, h = 0, 0, 0, 0
        if contours != 0:
            for contour in contours:
                if cv2.contourArea(contour) > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x,y), ((x+w), (y+h)), (255, 0, 0), 2)
                    break

            x_c = ((2 * x) + w) / 2
            y_c = ((2 * y) + h) / 2
            center = (x_c, y_c)

            cv2.circle(frame, (int(x_c), int(y_c)), 5, (255, 0, 0), cv2.FILLED)
            cv2.putText(red, str(center), (x, y), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

            cv2.circle(red, (int(x_c), int(y_c)), 5, (255, 0, 0), cv2.FILLED)
            cv2.putText(frame, str(center), (x, y), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

            #print("[INFO].. center is calculated", center) 
            center = rosRed()
            rospy.loginfo(image_callback) 


        cv2.imshow("mask", red)
        cv2.imshow("red", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


pub = rospy.Publisher("color_detection", Image, queue_size = 10)
pub.publish(image_callback)






