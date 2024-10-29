#!/usr/bin/env python3
import sys
#import rospy

import rclpy 
from rclpy.node import Node

from geometry_msgs.msg import Twist
import numpy as np
import time 
from geometry_msgs.msg import PoseStamped
import math
from collections import defaultdict


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
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.new_pose_received = False
        self.subscription = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.pose_callback,
            10)     
        # Dictionary with key being frame_id and value being a list [x, y, theta] of the april tag
        self.tags = {'1': [x1, z1, t1], '2': [x2, z2, t2], '3': [x3, z3, t3], '4': [x4, z4, t4], '5': [x5, z5, t5]}
        self.position_history = []

    def pose_callback(self, msg):
        x = msg.pose.position.x
        z = msg.pose.position.z
        x_ang = msg.pose.orientation.x
        y_ang = msg.pose.orientation.y
        z_ang = msg.pose.orientation.z
        w_ang = msg.pose.orientation.w
        frame_id = msg.header.frame_id
        self.current_state = self.calc_curr_state(x, z, x_ang, y_ang, z_ang, w_ang, frame_id)
        self.new_pose_received = True

    def wait_for_new_pose(self, update_value):
        timeout = 1
        start_time = time.time()

        while not self.new_pose_received:
            rclpy.spin_once(self)
            time.sleep(0.05)

            if time.time() - start_time > timeout:
                self.current_state += update_value
                break

        self.new_pose_received = False

    def calc_curr_state(self, x_det, z_det, x_ang, y_ang, z_ang, w_ang, frame_id):
        april_tag = self.tags[frame_id]
        
        trat = math.atan2(2 * (w_ang*y_ang - x_ang*z_ang), 1 - 2 * (y_ang*y_ang + z_ang*z_ang)) #calcutate pitch

        tro = april_tag[2] + trat
        tro = (tro + math.pi) % (2 * math.pi) - math.pi # scale to range

        xrat = x_det * np.cos(tro) - z_det * np.sin(tro)
        zrat = x_det * np.sin(tro) + z_det * np.cos(tro)

        xor = april_tag[0] - xrat
        zor = april_tag[1] - zrat

        return np.array([xor, zor, tro])

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

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
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

    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    rclpy.init()

    
    # rospy.init_node("hw1")
    #pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    # waypoint = np.array([[0.0,0.0,0.0], 
    #                      [-1.0/2,0.0,0.0],
    #                      [-1.0/2,1.0,np.pi.0],
    #                      [0.0,0.0,0.0]])
    waypoint = np.array([[0.0,0.0,0.0], 
                         [-1.0,-1.0,0.0], [-1, -1, 0], [-1, -1, np.pi]])

    # init pid controller
    pid = PIDcontroller(0.02, 0.005,0.005)
    print("kp", pid.Kp, "ki", pid.Ki, "kd", pid.Kd)
    time.sleep(3)

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for wp in waypoint:
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(pid.current_state)
        # publish the twist
        pid.publisher_.publish(genTwistMsg(coord(update_value, pid.current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        pid.wait_for_new_pose()
        print("current_state = ", pid.current_state)
        pid.position_history.append([pid.current_state[0], pid.current_state[1]])
        # current_state += update_value
        while rclpy.ok() and (np.linalg.norm(pid.getError(pid.current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(pid.current_state)
            # publish the twist
            pid.publisher_.publish(genTwistMsg(coord(update_value, pid.current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            # current_state += update_value
            pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
            pid.wait_for_new_pose()
            print("current_state = ", pid.current_state)

    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    print(pid.x_position_history)
    print(pid.z_position_history)