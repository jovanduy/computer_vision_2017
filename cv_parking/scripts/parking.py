#!/usr/bin/env python

""" This is a script that navigates the neato to park in an identified parking spot. """

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import deepcopy


class ParkingAgent(object):
    """ This script will navigate the neato into a parking spot. """


    def __init__(self):
        """ Initialize the parking agent """
        rospy.init_node('parking_agent')
        self.ParkingSpotRecognizer = ParkingSpotRecognizer()
        self.status = NOT_ALIGNED 
    
    def park(self):
        #send twist to move the neato forward
        #stop moving forward when lidar detects sufficiently close wall
        pass

    def align(self):
        pass
        
    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #detect a parking spot using the functions from our recognizer class
            #call the park function to navigate the neato into that spot
            if self.ParkingSpotRecognizer.dst:
                pass
            else:
                pass
            r.sleep()

if __name__ == '__main__':
    node = ParkingAgent()
    node.run()
