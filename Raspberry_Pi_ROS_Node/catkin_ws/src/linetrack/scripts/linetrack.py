#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import numpy as np
import math
import roslib
import sys
import rospy
from cv_bridge import CvBridge ,CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

cap = cv2.VideoCapture(0)

def detect_line():
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        ret, cv_image = cap.read()
        if not ret:  
            rospy.logerr("can not open camera!!!")
            break
        #cv2.imshow("img",cv_image)

        height, width, channels = cv_image.shape
        descentre = 50
        rows_to_watch = 100
        #crop_img = cv_image
        crop_img = cv_image[(height)/4 + descentre:(height)/4 + (descentre+rows_to_watch)][1:width]
        #convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img,cv2.COLOR_RGB2HSV)
        #cv2.imshow("HSV",hsv)
        # Red colour in HSV

        '''
        # If your line color changes, it's not a red color,you need to change the HSV[min,max] value in np.array[ , , ]
        
        '''
        #lower_red = np.array([54,45,0])
        #upper_red = np.array([255,255,255])

        lower_red = np.array([35,43,46])
        upper_red = np.array([77,255,255])
        #Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv,lower_red,upper_red)
        #cv2.imshow("MASK",mask)
        no_red = cv2.countNonZero(mask)
        print("points is " + str(no_red))
        #Bitwise-and musk and original image
        res = cv2.bitwise_and(crop_img,crop_img,mask = mask)
        #cv2.imshow("RES",res)

        #Calculate centroid of the blob of binary image using imageMoments
        m = cv2.moments(mask,False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
                cx, cy = height/2, width/2

        #Draw the centroid in the resultut image
        cv2.circle(res , (int(cx) , int(cy)) , 20 , (0,255,0) , 2)
        #print(cx)

        error_x = cx - width / 2      # error_x >0 :left   ||   error_x <0 : right

        twist_object = Twist()
        twist_object.linear.x = 0.05
        twist_object.angular.z = error_x / 80
        if ((math.fabs(twist_object.angular.z)) < 10 and (no_red > 20000)):
            cmd_vel_pub.publish(twist_object)
            rospy.loginfo("ANGULAR VALUE SENT ===>"+str(twist_object.angular.z))
        else:
            twist_object.linear.x = 0
            rospy.loginfo("Out of range Stop!!! " + str(twist_object.angular.z))
            twist_object.angular.z = 0
            cmd_vel_pub.publish(twist_object)

        #Publish detected line image 
        try:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', mask)[1]).tostring()
            image_pub.publish(msg)
        except CvBridgeError as e:
            print(e)
        cv2.waitKey(5)
        rate.sleep()
    cap.release()
    rospy.logerr("cam closed!!!")


if __name__ == '__main__':
    rospy.init_node('linetrack',anonymous=True)
    cmd_vel_pub = rospy.Publisher('joy_teleop/cmd_vel',Twist,queue_size = 10)
    image_pub = rospy.Publisher("line_detect_image/compressed",CompressedImage,queue_size=1)
    cvBridge = CvBridge()

    try:
        detect_line()
    except rospy.ROSInterruptException:
        cap.release()
        rospy.logerr("cam closed!!!")
        pass
