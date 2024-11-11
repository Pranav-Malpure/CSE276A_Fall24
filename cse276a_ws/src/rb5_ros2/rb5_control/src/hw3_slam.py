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
# import matplotlib.pyplot as plt

r = 0.025 # radius of the wheel
lx = 0.055 # half of the distance between front wheel and back wheel
ly = 0.07 
delta_t = 0.1
calibration_x = 180
calibration_y = 100
calibration_ang = 60


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
            1)     
        # Dictionary with key being frame_id and value being a list [x, y, theta] of the april tag
        # self.tags = {'6': [0, 1, np.pi/2], '2': [x2, z2, t2], '3': [x3, z3, t3], '4': [x4, z4, t4], '5': [x5, z5, t5]}
        # self.tags = {'4': [1, 0, 0], '6':[1, 1, 0], '5':[0.5, 1.5, np.pi/2], '7':[-0.45, 1, -np.pi],'2': [-0.41, 0, -np.pi], '1':[0, -0.5, -np.pi/2]}
        self.callback_data = []
        self.position_history = []
        self.detected_tag = []

    def pose_callback(self, msg):
        x = msg.pose.position.x
        z = msg.pose.position.z
        x_ang = msg.pose.orientation.x
        y_ang = msg.pose.orientation.y
        z_ang = msg.pose.orientation.z
        w_ang = msg.pose.orientation.w
        frame_id = msg.header.frame_id
        
        # z = z - np.sign(z)*(np.abs(z)-0.375)/0.125 # correcting for z error caused by april tag
        
        self.callback_data = [x, z, frame_id]
        # self.current_state = self.calc_curr_state(x, z, x_ang, y_ang, z_ang, w_ang, frame_id)
        # self.new_pose_received = True

    def get_measurement(self, kf):
        rclpy.spin_once(self)
        # print("callback data", self.callback_data)
        theta = (kf.state_update[2])   # TODO: have to bound this in -pi to pi
        # print("callback data", self.callback_data)
        if self.callback_data[2] in self.detected_tag:
            
            kf.z[int(self.callback_data[2])*2 - 1] = kf.state_update[0] + (self.callback_data[0]*np.cos(theta) - self.callback_data[1]*np.sin(theta))
            kf.z[int(self.callback_data[2])*2] = kf.state_update[1] + (self.callback_data[0]*np.sin(theta) + self.callback_data[1]*np.cos(theta))
        else:
            
            self.detected_tag.append(self.callback_data[2])
            kf.z[int(self.callback_data[2])*2 - 1] = kf.state_update[0] + (self.callback_data[0]*np.cos(theta) - self.callback_data[1]*np.sin(theta))
            kf.z[int(self.callback_data[2])*2] = kf.state_update[1] + (self.callback_data[0]*np.sin(theta) + self.callback_data[1]*np.cos(theta))

        print("z", kf.z[int(self.callback_data[2])*2 - 1], kf.z[int(self.callback_data[2])*2])

class KalmanFilter():
    def __init__(self):
        self.F = np.identity(53)
        self.G = np.zeros((53, 3)) 
        self.G[0] = [1,0,0]
        self.G[1] = [0,1,0] 
        self.G[2] = [0,0,1]
        self.G = delta_t*self.G
        # self.G = self.G*delta_t*r*0.25
        
        self.variance = 1000*np.identity(53)
        self.variance[0][0] = 0
        self.variance[1][1] = 0
        self.variance[2][2] = 0

        self.Q = np.zeros((53, 53))

        self.K_t = np.zeros((53, 50))

        self.H = np.zeros((50, 53))
        for i in range(50):
            self.H[i][i+3] = 1
        
        self.z = np.zeros((50, 1))

        self.variance_update = np.zeros((53, 53))

        self.state = np.zeros((53, 1))
        self.state_update = np.zeros((53, 1))

        self.R = np.zeros((50, 50))



    def predict(self, u):
        self.state_update = np.dot(self.F, self.state) + np.dot(self.G,u)
        # print("u", u)
        # print("G.u", np.dot(self.G, u))
        # print("state update", self.state_update)
        self.variance_update = np.dot(np.dot(self.F, self.variance), self.F.T) + self.Q


    def update(self):
        self.K_t = np.dot( np.dot(self.variance_update, self.H.T), np.linalg.pinv(np.dot( np.dot(self.H, self.variance_update), self.H.T)  + self.R) )
        # print("K_t", self.K_t)
        self.state = self.state_update + np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))

        self.variance = np.dot(np.identity(53) - np.dot(self.K_t, self.H), self.variance)

        # return self.next_state



    


def main():
    rclpy.init()

    kf = KalmanFilter()
    current_state = np.array([0, 0, 0])
    pid = PIDcontroller(0.02, 0, 0.075)

    # move in a square path of 1.5m side
    for i in range(4):
        while(True):
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.1
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            pid.publisher_.publish(twist_msg)

            time.sleep(delta_t)
            print("moving forward")
            # calculating the input vector below

            # desired_twist = np.array([[-calibration_x*twist_msg.linear.x], [calibration_y*twist_msg.linear.y], [calibration_ang*twist_msg.angular.z]])
            # jacobian_matrix = np.array([[1, -1, -(lx + ly)],
            #                          [1, 1, (lx + ly)],
            #                          [1, 1, -(lx + ly)],
            #                          [1, -1, (lx + ly)]])/r
            
            # input = np.dot(jacobian_matrix, desired_twist)

            input = np.array(([-calibration_x*twist_msg.linear.x/360], [calibration_y*twist_msg.linear.y/5], [calibration_ang*twist_msg.angular.z]))
            twist_msg.linear.y = 0.0
            # print("input", input)
            # have to check below parameters if they are actually angular velocities
            kf.predict(input) # have to correct this input according to the kinematic model and rewrite

            
            pid.publisher_.publish(twist_msg)
            
            time.sleep(1.5)
            # for j in range(25):
            #     pid.get_measurement(kf)
            pid.get_measurement(kf)
            
            print(kf.z[7], kf.z[8])
            
            kf.update() 
            
            print(kf.state[0], kf.state[1], kf.state[2], kf.state[10], kf.state[11], kf.state[12], kf.state[13])
            
            # if square side complete, break # TODO

            
if __name__ == '__main__':
    print("starting")
    main()