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

# status for parking fsm
NOT_ALIGNED = 0
ALIGNED = 1
PARKING = 2
PARKED = 3

class ParkingAgent(object):
    
    def __init__(self):
        """ Initialize the parking agent """
        rospy.init_node('parking_agent')
        self.ParkingSpotRecognizer = ParkingSpotRecognizer()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.twist = None
        self.status = NOT_ALIGNED # default status 
        self.state = None # sub-status in parking
        self.time = rospy.Time.now()
        self.speed_param = 0.07
        # threshold distance in x, y direction to be considered aligned
        self.x_threshold = 0.02
        self.y_threshold = 0.8

    
    def park(self):
        # guiding neato to move forward to neato length and stop
        self.time = rospy.Time.now()
        self.twist = Twist(linear = Vector3(1,0,0), angular=Vector3(0,0,0))
        self.publisher.publish(self.twist)
        while (rospy.Time.now() - self.time <= rospy.Duration(2.4)):
            # move forward at full speed for 2.4 seconds
            pass
        print "stopping"
        self.stop()
        self.status = PARKED
    
    def turn(self):
        # turn towards the spot and turn back to make sure the neato is facing the spot straight
        x, y = self.x, self.y
        turn_param = 4
        self.time = rospy.Time.now()
        self.twist = Twist(linear = Vector3(y*self.speed_param*15*abs(x),0,0), angular=Vector3(0,0,-x*turn_param*1.1))
        while (rospy.Time.now() - self.time <= rospy.Duration(1)):
            # turn to spot
            self.publisher.publish(self.twist)
        self.time = rospy.Time.now()
        self.twist = Twist(linear = Vector3(y*self.speed_param*3*abs(x),0,0), angular=Vector3(0,0,x*turn_param))
        while (rospy.Time.now() - self.time <= rospy.Duration(0.95)):
            # turn back
            self.publisher.publish(self.twist)
        self.twist = Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0))
        self.publisher.publish(self.twist)

    def stop(self):
        # stop the neato
        self.publisher.publish(Twist(linear = Vector3(0,0,0), angular=Vector3(0,0,0)))

    def back(self):
        # move back
        self.publisher.publish(Twist(linear = Vector3(-self.speed_param*self.y*2,0,0), angular=Vector3(0,0,0)))

    def forward(self):
        # go forward
        self.publisher.publish(Twist(linear = Vector3(self.speed_param*self.y,0,0), angular=Vector3(0,0,0)))
        
    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        rospy.on_shutdown(self.stop)
        self.state = self.stop # default sub-state
        while not rospy.is_shutdown():
            # first wait for the result of checking if the spot is empty
            if self.ParkingSpotRecognizer.is_spot_occupied is None:
                print "Checking whether the spot is empty."
            else:
                if self.ParkingSpotRecognizer.is_spot_occupied:
                    # if the spot is occupied, stop the node
                    print "Spot is occupied! Choose a different spot."
                    rospy.signal_shutdown("Spot is occupied! Choose a different spot.")
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
                                # if in the x direction the neato is aligned, set status to aligned
                                self.status = ALIGNED
                            else:
                                if y < self.y_threshold:
                                    # if in the x direction the neato isn't aligned and there isn't 
                                    # enough distance to the spot to make the turn, move back
                                    self.state = self.back
                                    print "back"
                                else:
                                    # if in the x direction the neato isn't aligned and there is enough 
                                    # distance to the spot to make the turn, turn towards the spot
                                    self.state = self.turn
                                    print "turn"
                        
                        if self.status == ALIGNED:
                            if y < self.y_threshold:
                                # if aligned and y distance to dst is within threshold, park
                                self.status = PARKING
                                print "parked"
                            else:
                                # else move forward
                                self.state = self.forward 
                                print "forward"  
                        
                        if self.status == PARKING:
                            print "parking"
                            self.state = self.park

                        if self.status == PARKED:
                            # after it's parked, shut down the node
                            self.state = self.stop
                            print "Done parking!"
                            rospy.signal_shutdown("Done parking.")

                        # execute function for current state
                        self.state()
                    
                    else:
                        pass
            r.sleep()

if __name__ == '__main__':
    node = ParkingAgent()
    node.run()


