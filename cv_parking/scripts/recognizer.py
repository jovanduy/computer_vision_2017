#!/usr/bin/env python

""" This script recognizes parking spots using opencv in ROS. """

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import deepcopy


class ParkingSpotRecognizer(object):
    """ This robot will recognize a parking spot. """


    def __init__(self):
        """ Initialize the parking spot recognizer """
        rospy.init_node('parking_spot_recognizer')
        self.cv_image = None                        # the latest image from the camera
        self.bridge = CvBridge()                    # used to convert ROS messages to OpenCV
        cv2.namedWindow('video_window')
        self.hsv_lb = np.array([0, 70, 60]) # hsv lower bound 
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
        self.hsv_ub = np.array([30, 255, 140]) # hsv upper bound
        
        
                
    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        #self.binary_image = cv2.inRange(self.hsv_image, (0,70,60), (30,255,140))
        self.binary_image = cv2.inRange(self.hsv_image, self.hsv_lb, self.hsv_ub)
        self.spot_delineators = self.find_delineators()
        if self.spot_delineators != None:
            cv2.line(self.cv_image,(self.spot_delineators[0][0],self.spot_delineators[0][1]),
                (self.spot_delineators[0][2],self.spot_delineators[0][3]),(0,255,0),2)
            cv2.line(self.cv_image,(self.spot_delineators[1][0],self.spot_delineators[1][1]),
                (self.spot_delineators[1][2],self.spot_delineators[1][3]),(0,255,0),2)
        
               
    def hough_lines(self):
       """ This function uses the Hough Line Transform function to identify and visualize lines in our binary image."""
       
       lines = cv2.HoughLinesP(self.binary_image, rho=5, theta=np.deg2rad(10), threshold=100, minLineLength=25, maxLineGap=0)
       lines_filtered = []
       if lines != None:
            for x1,y1,x2,y2 in lines[0]:
                if y1 >100 and y2 > 100 and abs(y1 - y2) > 10:
                    cv2.line(self.cv_image,(x1,y1),(x2,y2),(0,0,255),2)
                    lines_filtered.append((x1,y1,x2,y2))
       return lines_filtered

   
    def find_delineators(self):
        """This function uses the pinhole camera model to determine the endpoints of a parking spot in 3d space."""
        lines = self.hough_lines()
        if len(lines) < 3:
            return
        # sorting by left to right in the image
        lines.sort(key = lambda x: x[0])
        endpoint1 = lines[0]
        endpoint2 = -1
        leftmostx = lines[0][0]
        x_range = 120
        index = 1
        while index < len(lines) and lines[index][0] - leftmostx < x_range :
            if endpoint1[1] < lines[index][1]:
                endpoint1 = lines[index]
            index += 1

        if index < len(lines):
            endpoint2 = lines[index]
            leftmostx = endpoint2[0]
            
            while index < len(lines) and lines[index][0] - leftmostx < x_range:
                if endpoint2[1] < lines[index][1]:
                    endpoint2 = lines[index]
                index += 1

        if endpoint2 != -1:
            return [endpoint1, endpoint2]
        else:
            return None

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # creates a window and displays the image for X milliseconds
                cv2.imshow('video_window', self.cv_image)
                cv2.imshow('binary', self.binary_image)
                cv2.waitKey(5)
            r.sleep()

if __name__ == '__main__':
    node = ParkingSpotRecognizer()
    node.run()
