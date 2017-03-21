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
        rospy.Subscriber("/camera/image_raw", Image, self.process_image)
                
    def process_image(self, msg):
        """ Process image messages from ROS and stash them in an attribute
            called cv_image for subsequent processing """
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.binary_image = cv2.inRange(self.hsv_image, (0,70,60), (30,255,140))
        outline = self.bounding_contour()
        cv2.drawContours(self.cv_image,outline,-1,(0,255,0),5)
        
    def bounding_contour(self):
        """
        Returns
        -------
        (left_top, right_bottom) where left_top and right_bottom are tuples of (x_pixel, y_pixel)
            defining topleft and bottomright corners of the bounding box
        """
        image = deepcopy(self.binary_image)
        im2, contours, hierarchy = cv2.findContours(image, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        cnt = contours[0]

        epsilon = 0.01*cv2.arcLength(cnt,True)

        approx = cv2.approxPolyDP(cnt,epsilon,True)
        
        print "APPROX:", approx
        print "size of approx", len(approx)
        return approx

    def spot_detected(self):
        #first detect whether the spot is empty
        #then use pinhole camera method to determine surface area of the spot
        #if surface area > threshold, return true or false,
        #or maybe return base_link coordinates of the corners of the parking spot so the neato can align itself and then drive in
        pass

    
    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.cv_image is None:
                # creates a window and displays the image for X milliseconds
                cv2.imshow('video_window', self.cv_image)
                cv2.imshow('hsv_window', self.hsv_image)
                cv2.imshow('binary', self.binary_image)
                cv2.waitKey(5)
            r.sleep()

if __name__ == '__main__':
    node = ParkingSpotRecognizer()
    node.run()