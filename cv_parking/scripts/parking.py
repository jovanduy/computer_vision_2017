#!/usr/bin/env python

""" This is a script that navigates the neato to park in an identified parking spot. """

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from copy import deepcopy
from recognizer import ParkingSpotRecognizer

NOT_ALIGNED = 0
ALIGNED = 1
PARKING = 2

class ParkingAgent(object):
    """ This script will navigate the neato into a parking spot. """


    def __init__(self):
        """ Initialize the parking agent """
        rospy.init_node('parking_agent')
        self.ParkingSpotRecognizer = ParkingSpotRecognizer()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.twist = None
        self.status = NOT_ALIGNED 
        self.state = None
        self.time = rospy.Time.now()
        self.speed_param = 0.2
        self.x_threshold = 0.03
        self.y_threshold = 0.8

    
    def park(self):
        #send twist to move the neato forward
        #stop moving forward when lidar detects sufficiently close wall
        self.twist = Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0))


    def align(self, x, y):
        if abs(x) < 0.08:
            if y < 0.15:
                self.twist = Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0))
                self.status = ALIGNED 
            else:
                # forward
                print "forward"
                self.twist = Twist(linear = Vector3(y*1,0,0), angular=Vector3(0,0,0))
        else:
            if y > 0.05:
                # turn
                print "reset and turning"
                self.time = rospy.Time.now()
                self.status = TURNING
                self.turn(x, y)
            else:
                # go back
                print "going back"
                self.twist = Twist(linear = Vector3(-y*0.8,0,0), angular=Vector3(0,0,0))

    
    def turn(self, x, y):
        turn_param = 4
        speed_param = 0.08
        self.twist = Twist(linear = Vector3(y*speed_param,0,0), angular=Vector3(0,0,x*turn_param))
        while (rospy.Time.now() - self.time <= rospy.Duration(0.5)):
            pass
        self.time = rospy.Time.now()
        self.twist = Twist(linear = Vector3(y*speed_param,0,0), angular=Vector3(0,0,-x*turn_param))
        while (rospy.Time.now() - self.time <= rospy.Duration(0.5)):
            pass
        self.twist = Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0))


    def stop(self):
        self.publisher.publish(Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0)))

    def back(self):
        self.publisher.publish(Twist(linear = Vector3(-self.speed_param*self.y,0,0), angular=Vector3(0,0,0)))

    def forward(self):
        self.publisher.publish(Twist(linear = Vector3(self.speed_param*self.y,0,0), angular=Vector3(0,0,0)))
        
    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        rospy.on_shutdown(self.stop)
        self.state = self.stop
        while not rospy.is_shutdown():
            #detect a parking spot using the functions from our recognizer class
            #call the park function to navigate the neato into that spot
            if self.ParkingSpotRecognizer.dst:
                x = self.ParkingSpotRecognizer.dst[0]
                y = self.ParkingSpotRecognizer.dst[1]
                self.x, self.y = x, y
                if self.status == NOT_ALIGNED:
                    if x < self.x_threshold:
                        self.status = ALIGNED
                    else:
                        if y < y_threshold:
                            self.state = self.back
                            print "back"
                        else:
                            self.state = self.turn
                            print "turn"
                
                if self.status == ALIGNED:
                    if y < self.y_threshold:
                        self.status = PARKING
                    else:
                        self.state = self.forward 
                        print "forward"  
                
                if self.status == PARKING:
                    self.state = self.park
                
                print self.state
                self.state()
            
            else:
                pass
            r.sleep()

if __name__ == '__main__':
    node = ParkingAgent()
    node.run()
