#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import roslib
import sys
import rospy
from cv_bridge import CvBridge ,CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

class Fire_detect(object):
    def __init__(self):
        self.cvBridge = CvBridge()
        self.sub_left_camera = rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback, queue_size = 1)
        self.image_pub = rospy.Publisher("fire_detect_image/compressed",CompressedImage,queue_size=1)

    # ROS Image's topic callback function
    def camera_callback(self, image_msg): 
        try:
            frame = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
                print(e)
        #cv2.imshow("cv_imcode_image",frame)

        blur = cv2.GaussianBlur(frame, (21, 21), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
 
        lower = [0, 0, 208]
        upper = [179, 255, 255]
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(hsv, lower, upper)
 
        output = cv2.bitwise_and(frame, hsv, mask=mask)
        no_red = cv2.countNonZero(mask)
        
        if int(no_red) > 1500:
            text = "Fire Detect"
            cv2.putText(frame, text, (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2.0, (0, 0, 255), 5)
            #cv2.imshow("output", frame)
        else:
            cv2.putText(frame, "No Fire", (200, 100), cv2.FONT_HERSHEY_COMPLEX, 2.0, (0, 255, 0), 5)
            #cv2.imshow("output", frame)
        print(int(no_red))

        #Publish detected line image 
        try:
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
            self.image_pub.publish(msg)
        except CvBridgeError as e:
            print(e)

def main():
    rospy.init_node('fire_detect_node',anonymous=True)
    Fire_detect_node = Fire_detect()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
