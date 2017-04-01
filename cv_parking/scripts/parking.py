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
PARKED = 3

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
        self.speed_param = 0.07
        self.x_threshold = 0.02
        self.y_threshold = 0.8

    
    def park(self):
        #send twist to move the neato forward
        #stop moving forward when lidar detects sufficiently close wall
        self.time = rospy.Time.now()
        self.twist = Twist(linear = Vector3(1,0,0), angular=Vector3(0,0,0))
        self.publisher.publish(self.twist)
        while (rospy.Time.now() - self.time <= rospy.Duration(2.4)):
            pass
        print "stopping"
        self.stop()
        self.status = PARKED


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

    
    def turn(self):
        x, y = self.x, self.y
        turn_param = 4
        self.time = rospy.Time.now()
        self.twist = Twist(linear = Vector3(y*self.speed_param*15*abs(x),0,0), angular=Vector3(0,0,-x*turn_param*1.1))
        while (rospy.Time.now() - self.time <= rospy.Duration(1)):
            self.publisher.publish(self.twist)
        self.time = rospy.Time.now()
        self.twist = Twist(linear = Vector3(y*self.speed_param*3*abs(x),0,0), angular=Vector3(0,0,x*turn_param))
        while (rospy.Time.now() - self.time <= rospy.Duration(0.95)):
            self.publisher.publish(self.twist)
        self.twist = Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0))
        self.publisher.publish(self.twist)

    # def turn(self, x, y):
    #     turn_param = 4
    #     self.time = rospy.Time.now()
    #     self.twist = Twist(linear = Vector3(y*self.speed_param*1.5,0,0), angular=Vector3(0,0,-x*turn_param))
    #     while (rospy.Time.now() - self.time <= rospy.Duration(1)):
    #         self.publisher.publish(self.twist)
    #     self.time = rospy.Time.now()
    #     self.twist = Twist(linear = Vector3(y*self.speed_param*0.3,0,0), angular=Vector3(0,0,x*turn_param))
    #     while (rospy.Time.now() - self.time <= rospy.Duration(0.9)):
    #         self.publisher.publish(self.twist)
    #     self.twist = Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0))
    #     self.publisher.publish(self.twist)

    def stop(self):
        self.publisher.publish(Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0)))

    def back(self):
        self.publisher.publish(Twist(linear = Vector3(-self.speed_param*self.y*2,0,0), angular=Vector3(0,0,0)))

    def forward(self):
        self.publisher.publish(Twist(linear = Vector3(self.speed_param*self.y,0,0), angular=Vector3(0,0,0)))
        
    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        rospy.on_shutdown(self.stop)
        self.state = self.stop
        while not rospy.is_shutdown():
            if self.ParkingSpotRecognizer.is_spot_occupied is None:
                print "Checking whether the spot is empty."
            if self.ParkingSpotRecognizer.is_spot_occupied is None:
                pass
            else:
                if self.ParkingSpotRecognizer.is_spot_occupied:
                    print "Spot is occupied! Choose a different spot."
                    rospy.signal_shutdown("Spot is occupied! Choose a different spot.")
                #detect a parking spot using the functions from our recognizer class
                #call the park function to navigate the neato into that spot
                else:
                    if self.ParkingSpotRecognizer.dst:
                        cv2.imshow('video_window', self.ParkingSpotRecognizer.cv_image)
                        cv2.imshow('binary', self.ParkingSpotRecognizer.binary_image)
                        cv2.waitKey(5)
                        x = self.ParkingSpotRecognizer.dst[0]
                        y = self.ParkingSpotRecognizer.dst[1]
                        self.x, self.y = x, y
                        if self.status == NOT_ALIGNED:
                            if abs(x) < self.x_threshold:
                                self.status = ALIGNED
                            else:
                                if y < self.y_threshold:
                                    self.state = self.back
                                    print "back"
                                else:
                                    self.state = self.turn
                                    print "turn"
                        
                        if self.status == ALIGNED:
                            if y < self.y_threshold:
                                self.status = PARKING
                                print "parked"
                            else:
                                self.state = self.forward 
                                print "forward"  
                        
                        if self.status == PARKING:
                            print "parking"
                            self.state = self.park

                        if self.status == PARKED:
                            self.state = self.stop
                            print "Done parking!"
                            rospy.signal_shutdown("Done parking.")

                        self.state()
                    
                    else:
                        pass
            r.sleep()

if __name__ == '__main__':
    node = ParkingAgent()
    node.run()
    # node.turn(0.1352527682466631, 2.247034385154258)

