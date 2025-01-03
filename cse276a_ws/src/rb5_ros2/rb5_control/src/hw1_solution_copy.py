#!/usr/bin/env python3
import sys
#import rospy

import rclpy 
from rclpy.node import Node

from geometry_msgs.msg import Twist
import numpy as np
import time 

"""
The class of the pid controller.
"""
class PIDcontroller(Node):
    def __init__(self, Kp, Ki, Kd):
        super().__init__('PID_Controller_NodePub')
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)
        print("created publisher")

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = np.array([self.Kp * e[0]/0.75, self.Kp * e[1], self.Kp * e[2]])
        # self.I = np.array([self.I[0] + self.Ki * e[0] * self.timestep, self.I[1] + self.Ki * e[1] * self.timestep , self.I[2] + self.Ki * e[2] * self.timestep/2]) 
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = np.array([self.Kd * (e[0] - self.lastError[0]), self.Kd * (e[1] - self.lastError[1]), self.Kd * (e[2] - self.lastError[2])])
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0.0
    twist_msg.angular.x = 0.0
    twist_msg.angular.y = 0.0
    twist_msg.angular.z = desired_twist[2]
    print(twist_msg)
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]-np.pi/2), np.sin(current_state[2]-np.pi/2), 0.0],
                  [-np.sin(current_state[2]-np.pi/2), np.cos(current_state[2]-np.pi/2), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    rclpy.init()

    
    # rospy.init_node("hw1")
    #pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    # waypoint = np.array([[0.0,0.0,0.0], 
    #                      [-1.0/2,0.0,0.0],
    #                      [-1.0/2,1.0/2,np.pi/2.0],
    #                      [-2.0/2,1.0/2,0.0],
    #                      [-2.0/2,2.0/2,-np.pi/2.0],
    #                      [-1.0/2,1.0/2,-np.pi/4.0],
    #                      [0.0,0.0,0.0]]) 
    waypoint = np.array([[0.0,0.0,0], [0, -1/2, np.pi/2]])
    # waypoint = np.array([[0.0,0.0,0.0], [-1/4,-1/4,0]])
    # waypoint = np.array([[0.0,0.0,0.0], [0.0, 0.0, np.pi/2.0]])

    # init pid controller
    pid = PIDcontroller(0.02, 0.015, 0.075)
    print("kp", pid.Kp, "ki", pid.Ki, "kd", pid.Kd)
    time.sleep(3)
    # init current state
    current_state = np.array([0.0,0.0,np.pi/2])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        print("update value",update_value)
        # publish the twist
        pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pid.publisher_.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.1)
            # update the current state
            current_state += update_value
            print("current_state = ", current_state)
            pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

            # print("update value",update_value)
            time.sleep(0.5)

    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

