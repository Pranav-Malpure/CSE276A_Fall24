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
import pickle


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
        self.current_state = np.array([0.0762, 0.0762, 0])
        self.new_pose_received = False
        self.subscription = self.create_subscription(
            PoseStamped, 
            '/april_poses',
            self.pose_callback,
            10)     
        # Dictionary with key being frame_id and value being a list [x, y, theta] of the april tag
        self.tags = {'2': [2.25*0.3048, -0.75*0.3048, -np.pi/2], '7': [4.5*0.3048, -0.75*0.3048, -np.pi/2], '6': [6.75*0.3048, -0.75*0.3048, -np.pi/2], '10': [9*0.3048, 1.5*0.3048, 0], '11': [9*0.3048, 3.75*0.3048, 0],
                     '12': [9*0.3048, (6.75-0.75)*0.3048, 0], '4': [6.75*0.3048, (9-0.75)*0.3048, np.pi/2], '1': [4.5*0.3048, (9-0.75)*0.3048, np.pi/2], '5': [2.25*0.3048, (9-0.75)*0.3048, np.pi/2]}

        self.position_history = []
        self.april_tag_data = dict()

    def pose_callback(self, msg):
        x = msg.pose.position.x
        z = msg.pose.position.z
        x_ang = msg.pose.orientation.x
        y_ang = msg.pose.orientation.y
        z_ang = msg.pose.orientation.z
        w_ang = msg.pose.orientation.w
        frame_id = msg.header.frame_id
        
        self.current_state = self.calc_curr_state(x, z, x_ang, y_ang, z_ang, w_ang, frame_id)
        self.april_tag_data[frame_id] = [x, z, w_ang]
        self.new_pose_received = True

    def wait_for_new_pose(self, update_value):
        timeout = 1
        start_time = time.time()

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
    return twist_msg

def coord(twist, current_state):
    # print("twist before",twist)
    J = np.array([[np.cos(current_state[2]-np.pi/2), np.sin(current_state[2]-np.pi/2), 0.0],
                  [-np.sin(current_state[2]-np.pi/2), np.cos(current_state[2]-np.pi/2), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    rclpy.init()
    
    waypoint = np.array([
    [1.143, 0.0762, 0.0],
    [2.2098, 0.0762, 0.0],
    [2.2098, 0.22860000000000003, 0.0],
    [1.143, 0.22860000000000003, 0.0],
    [0.0762, 0.22860000000000003, 0.0],
    [0.0762, 0.381, 0.0],
    [1.143, 0.381, 0.0],
    [2.2098, 0.381, 0.0],
    [2.2098, 0.5334, 0.0],
    [1.143, 0.5334, 0.0],
    [0.0762, 0.5334, 0.0],
    [0.0762, 0.6858000000000001, 0.0],
    [1.143, 0.6858000000000001, 0.0],
    [2.2098, 0.6858000000000001, 0.0],
    [2.2098, 0.8382000000000001, 0.0],
    [1.143, 0.8382000000000001, 0.0],
    [0.0762, 0.8382000000000001, 0.0],
    [0.0762, 0.9906, 0.0],
    [1.143, 0.9906, 0.0],
    [2.2098, 0.9906, 0.0],
    [2.2098, 1.143, 0.0],
    [1.143, 1.143, 0.0],
    [0.0762, 1.143, 0.0],
    [0.0762, 1.2954, 0.0],
    [1.143, 1.2954, 0.0],
    [2.2098, 1.2954, 0.0],
    [2.2098, 1.4478, 0.0],
    [1.143, 1.4478, 0.0],
    [0.0762, 1.4478, 0.0],
    [0.0762, 1.6002, 0.0],
    [1.143, 1.6002, 0.0],
    [2.2098, 1.6002, 0.0],
    [2.2098, 1.7526000000000002, 0.0],
    [1.143, 1.7526000000000002, 0.0],
    [0.0762, 1.7526000000000002, 0.0],
    [0.0762, 1.905, 0.0],
    [1.143, 1.905, 0.0],
    [2.2098, 1.905, 0.0],
    [2.2098, 2.0574, 0.0],
    [1.143, 2.0574, 0.0],
    [0.0762, 2.0574, 0.0],
    [0.0762, 2.2098, 0.0],
    [1.143, 2.2098, 0.0],
    [2.2098, 2.2098, 0.0]])
    

    pid = PIDcontroller(0.02, 0, 0.075)
    print("kp", pid.Kp, "ki", pid.Ki, "kd", pid.Kd)
    time.sleep(3)
    pid.current_state = np.array([6*0.3048,2*0.3048,np.pi/2])

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
        time.sleep(0.05)
        # update the current state
        pid.wait_for_new_pose(update_value)
        print("current_state = ", pid.current_state)
        pid.position_history.append([pid.current_state[0], pid.current_state[1], pid.current_state[2]])

        
        if (np.linalg.norm(pid.getError(pid.current_state, wp)[:2]) < 0.03):
            pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
            print("inside angle regime_initial")
            while rclpy.ok() and abs(pid.getError(pid.current_state, wp)[2]) > 0.03: # check the error between current state and current way point
                # calculate the current twist
                update_value = pid.update(pid.current_state)

                angle_twist_msg = genTwistMsg(coord(update_value, pid.current_state))
                angle_twist_msg.linear.x = 0.0
                angle_twist_msg.linear.y = 0.0
                pid.publisher_.publish(angle_twist_msg)
                time.sleep(0.05)
                pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
                pid.wait_for_new_pose(update_value)

                print("current_state = ", pid.current_state)
                time.sleep(0.5)
                pid.position_history.append([pid.current_state[0], pid.current_state[1], pid.current_state[2]])



        while rclpy.ok() and (np.linalg.norm(pid.getError(pid.current_state, wp)[:2]) > 0.15): # check the error between current state and current way point
            print("WAYPOINT NUMBER", wp)
            if abs(pid.getError(pid.current_state, wp)[0]) < 0.1:
                x_reached = True
                print("reached x")
                print('X ERROR', pid.getError(pid.current_state, wp)[0])
            else:
                x_reached = False
            if abs(pid.getError(pid.current_state, wp)[1]) < 0.1:
                z_reached = True
                print('Z ERROR', pid.getError(pid.current_state, wp)[1])
                print("reached z")
            else:
                z_reached = False

            print("current error = ", (pid.getError(pid.current_state, wp)))
            # calculate the current twist
            update_value = pid.update(pid.current_state)
            # publish the twist

            if x_reached:
                # twist_msg.linear.x = 0.0
                update_value[0] = 0.0
            if z_reached:
                # twist_msg.linear.y = 0.0
                update_value[1] = 0.0
            twist_msg = genTwistMsg(coord(update_value, pid.current_state))
            twist_msg.angular.z = 0.0
            print("twist msg",twist_msg)

            pid.publisher_.publish(twist_msg)
            time.sleep(0.05)
            
            pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
            pid.wait_for_new_pose(update_value)
            print("current_state = ", pid.current_state)
            time.sleep(0.5)
            pid.position_history.append([pid.current_state[0], pid.current_state[1], pid.current_state[2]])

            print("angle enter error", (np.linalg.norm(pid.getError(pid.current_state, wp)[:2])))
            if (np.linalg.norm(pid.getError(pid.current_state, wp)[:2]) < 0.15):
                pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
                print("inside angle regime")
                while rclpy.ok() and abs(pid.getError(pid.current_state, wp)[2]) > 0.1: # check the error between current state and current way point
                    # calculate the current twist
                    update_value = pid.update(pid.current_state)

                    
                    angle_twist_msg = genTwistMsg(coord(update_value, pid.current_state))
                    angle_twist_msg.linear.x = 0.0
                    angle_twist_msg.linear.y = 0.0
                    pid.publisher_.publish(angle_twist_msg)
                    #print(coord(update_value, current_state))
                    time.sleep(0.05)
                    # update the current state
                    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))
                    pid.wait_for_new_pose(update_value)

                    print("current_state = ", pid.current_state)
                    time.sleep(0.5)
                    pid.position_history.append([pid.current_state[0], pid.current_state[1], pid.current_state[2]])


        # Save the list to a file in every loop
        with open('pid_position_history.pkl', 'wb') as f:
            pickle.dump(pid.position_history, f)


    # stop the car and exit
    pid.publisher_.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

    # Save the list to a file
    with open('pid_position_history.pkl', 'wb') as f:
        pickle.dump(pid.position_history, f)

    print(pid.position_history)
