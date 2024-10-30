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
        self.timestep = 0.2
        self.maximumValue = 0.1
        self.publisher_ = self.create_publisher(Twist, '/twist', 10)
        print("created publisher")
        self.current_state = np.array([0.0, 0.0, 0])
        self.new_pose_received = False
        self.subscription = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.pose_callback,
            10)     
        # Dictionary with key being frame_id and value being a list [x, y, theta] of the april tag
        # self.tags = {'6': [0, 1, np.pi/2], '2': [x2, z2, t2], '3': [x3, z3, t3], '4': [x4, z4, t4], '5': [x5, z5, t5]}
        self.tags = {'4': [0, 1, -np.pi], '6':[1, 1, -np.pi]}

        self.position_history = []

    def pose_callback(self, msg):
        x = msg.pose.position.x
        z = msg.pose.position.z
        x_ang = msg.pose.orientation.x
        y_ang = msg.pose.orientation.y
        z_ang = msg.pose.orientation.z
        w_ang = msg.pose.orientation.w
        frame_id = msg.header.frame_id
        # z = z - (z-37.5)/12.5 # correcting for z error.
        self.current_state = self.calc_curr_state(x, z, x_ang, y_ang, z_ang, w_ang, frame_id)
        self.new_pose_received = True

    def wait_for_new_pose(self, update_value):
        timeout = 1
        start_time = time.time()

        # while not self.new_pose_received:
        #     rclpy.spin_once(self)
        #     # time.sleep(0.05)

        #     if time.time() - start_time > timeout:
        #         self.current_state += update_value
        #         break
        if not self.new_pose_received:
            rclpy.spin_once(self)
        else:
            self.current_state += update_value
        self.new_pose_received = False

    def calc_curr_state(self, x_det, z_det, x_ang, y_ang, z_ang, w_ang, frame_id):
        april_tag = self.tags[frame_id]
        
        trat = math.atan2(2 * (w_ang*y_ang - x_ang*z_ang), 1 - 2 * (y_ang*y_ang + z_ang*z_ang)) #calcutate pitch

        tro = april_tag[2] + trat
        tro = (tro + math.pi) % (2 * math.pi) - math.pi # scale to range

        xrat = x_det * np.cos(tro - np.pi/2) - z_det * np.sin(tro - np.pi/2)
        zrat = x_det * np.sin(tro - np.pi/2) + z_det * np.cos(tro - np.pi/2)

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
        # """
        # calculate the update value on the state based on the error between current state and target state with PID.
        # """
        # e = self.getError(currentState, self.target)

        # P = self.Kp * e
        # self.I = self.I + self.Ki * e * self.timestep 
        # I = self.I
        # D = self.Kd * (e - self.lastError)
        # result = P + I + D

        # self.lastError = e

        # # scale down the twist if its norm is more than the maximum value. 
        # resultNorm = np.linalg.norm(result)
        # if(resultNorm > self.maximumValue):
        #     result = (result / resultNorm) * self.maximumValue
        #     self.I = 0.0

        # return result
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
    # print(twist_msg)
    return twist_msg

def coord(twist, current_state):
    # J = np.array([[np.cos(current_state[2]-np.pi/2), np.sin(current_state[2]-np.pi/2), 0.0],
    #               [-np.sin(current_state[2]-np.pi/2), np.cos(current_state[2]-np.pi/2), 0.0],
    #               [0.0,0.0,1.0]])
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
    # waypoint = np.array([[0.0,0.0,0.0], 
                        #  [-1.0,-1.0,0.0], [-1, -1, 0], [-1, -1, np.pi]])

    waypoint = np.array([[0,0,0],[0,1/2,0], [1, 1/2, 0]])
    # init pid controller
    pid = PIDcontroller(0.02, 0, 0.075)
    print("kp", pid.Kp, "ki", pid.Ki, "kd", pid.Kd)
    time.sleep(3)
    pid.current_state = np.array([0,0,0])

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
        # print("update_value",update_value)
        # print(genTwistMsg(coord(update_value, pid.current_state)))
        pid.publisher_.publish(genTwistMsg(coord(update_value, pid.current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        pid.wait_for_new_pose(update_value)
        # print("update value",update_value)
        print("current_state = ", pid.current_state)
        pid.position_history.append([pid.current_state[0], pid.current_state[1], pid.current_state[2]])
        # current_state += update_value
        while rclpy.ok() and (np.linalg.norm(pid.getError(pid.current_state, wp)[:2]) > 0.05): # check the error between current state and current way point
            if abs(pid.getError(pid.current_state, wp)[0]) < 0.05:
                x_reached = True
                print("reached x")
                print('X ERROR', pid.getError(pid.current_state, wp)[0])

            else:
                x_reached = False
            if abs(pid.getError(pid.current_state, wp)[1]) < 0.05:
                z_reached = True
                print('Z ERROR', pid.getError(pid.current_state, wp)[1])
                print("reached z")
            else:
                z_reached = False

            if abs(pid.getError(pid.current_state, wp)[2]) < 0.2:
                angle_reached = True
                print('ANGLE ERROR', pid.getError(pid.current_state, wp)[2])
                print("reached angle")
            else:
                angle_reached = False


            print("current error = ", (pid.getError(pid.current_state, wp)))
            # calculate the current twist
            update_value = pid.update(pid.current_state)
            # publish the twist
            # print("update_value",update_value)

            # print(genTwistMsg(coord(update_value, pid.current_state)))
            twist_msg = genTwistMsg(update_value)
            # twist_msg.angular.z = 0.0
            if x_reached:
                twist_msg.linear.x = 0.0
            if z_reached:
                twist_msg.linear.z = 0.0
            if angle_reached:
                twist_msg.angular.z = 0.0
            pid.publisher_.publish(twist_msg)
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            # current_state += update_value
            
            pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
            pid.wait_for_new_pose(update_value)
            print("current_state = ", pid.current_state)
            # print("update value",update_value)
            time.sleep(1)
            # time.sleep(2)
            if (np.linalg.norm(pid.getError(pid.current_state, wp)[:2]) < 0.05):
                pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
                print("inside angle regime")
                while rclpy.ok() and abs(pid.getError(pid.current_state, wp)[2]) > 0.1: # check the error between current state and current way point
                    # calculate the current twist
                    update_value = pid.update(pid.current_state)

                    
                    # publish the twist
                    # print("update_value",update_value)

                    # print(genTwistMsg(coord(update_value, pid.current_state)))
                    angle_twist_msg = genTwistMsg(coord(update_value, pid.current_state))
                    angle_twist_msg.linear.x = 0.0
                    angle_twist_msg.linear.y = 0.0
                    pid.publisher_.publish(angle_twist_msg)
                    #print(coord(update_value, current_state))
                    time.sleep(0.05)
                    # update the current state
                    # current_state += update_value
                    # pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
                    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
                    pid.wait_for_new_pose(update_value)

                    print("current_state = ", pid.current_state)
                    # print("update value",update_value)
                    time.sleep(1)
                    # time.sleep(2)

    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    print(pid.position_history)
