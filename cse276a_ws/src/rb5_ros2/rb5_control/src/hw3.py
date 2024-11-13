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
delta_t = 0.02
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
        self.callback_data = []
        self.position_history = []

    def pose_callback(self, msg):
        x = msg.pose.position.x
        z = msg.pose.position.z
        x_ang = msg.pose.orientation.x
        y_ang = msg.pose.orientation.y
        z_ang = msg.pose.orientation.z
        w_ang = msg.pose.orientation.w
        frame_id = msg.header.frame_id
        
        self.callback_data = [x, z, frame_id]

    def get_measurement(self, kf):
        rclpy.spin_once(self)
        # print("callback data", self.callback_data)
        # theta = (kf.state_update[2])   # TODO: have to bound this in -pi to pi
        theta = kf.state[2]  # TODO: have to bound this in -pi to pi, and have to chose either this or above one
        # print("callback data", self.callback_data)
        if self.callback_data[2] in kf.detected_tag:
            kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 3] = np.cos(theta)
            kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = -np.sin(theta)

            kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 3] = np.sin(theta)
            kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = np.cos(theta)
            # kf.H[int(self.callback_data[2])*2 - 1][int(self.callback_data[2])*2 - 1 + 3] = 1
            # kf.H[int(self.callback_data[2])*2][int(self.callback_data[2])*2 + 3] = 1
        else:
            kf.detected_tag.append(self.callback_data[2])
            kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 3] = np.cos(theta)
            kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = -np.sin(theta)

            kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 3] = np.sin(theta)
            kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = np.cos(theta)


            kf.z[(int(self.callback_data[2]) - 1)*2] = self.callback_data[0]
            kf.z[(int(self.callback_data[2]) - 1)*2 + 1] = self.callback_data[1]
            if kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] == 0 and kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] == 0:
                print('inside new tag')
                kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] = self.callback_data[0]*np.cos(theta) + self.callback_data[1]*np.sin(theta)  + kf.state_update[0] # TODO: Add angle transformation of axes
                kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] = -self.callback_data[0]*np.sin(theta) + self.callback_data[1]*np.cos(theta) + kf.state_update[1] # TODO: Add angle transformation of axes
                kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3] = 1e-2
                kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1] = 1e-2
                print("variance update in get_measurement", kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3], kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1])
        # print('state update after AT', kf.state_update[0], kf.state_update[1], kf.state_update[2], kf.state_update[10], kf.state_update[11])


class KalmanFilter():
    def __init__(self):
        self.F = np.identity(53)
        self.G = np.zeros((53, 3)) 
        self.G[0] = [1,0,0]
        self.G[1] = [0,1,0] 
        self.G[2] = [0,0,1]
        self.G = delta_t*self.G
        # self.G = self.G*delta_t*r*0.25
        
        # self.variance = 1000*np.identity(53)
        self.variance = 1000*np.identity(53)
        self.variance[0][0] = 0
        self.variance[1][1] = 0
        self.variance[2][2] = 0

        # self.Q = np.zeros((53, 53))
        self.Q = np.identity(53)
        self.Q[0][0] = 0.01
        self.Q[1][1] = 0.02
        self.Q[2][2] = 0.0

        # self.K_t = np.zeros((53, 50))

        # self.S = np.zeros((50, 50))

        self.H_core = np.zeros((50, 53))
        for i in range(50):
            if i % 2 == 0:
                self.H_core[i][0] = -1
            else:
                self.H_core[i][1] = -1 

        self.H = np.zeros((50, 53)) # H.s is actually where you think the april tag is, and z is actually where it is. it should be in robot frame
        for i in range(50):
            if i % 2 == 0:
                self.H[i][0] = -1
            else:
                self.H[i][1] = -1 # subtract the x and y of the robot to get where the april tag can be
        
        self.z = np.zeros((50, 1))

        self.variance_update = np.zeros((53, 53))

        self.state = np.zeros((53, 1))

        self.state_update = np.zeros((53, 1))

        self.R = np.identity(50)*1e-2

        self.detected_tag = []

    def predict(self, u):        
        self.state_update = np.dot(self.F, self.state) + np.dot(self.G,u)
        # print("u", u)
        # print("G.u", np.dot(self.G, u))

        print("state update before AT", self.state_update[0], self.state_update[1], self.state_update[2], self.state_update[9], self.state_update[10])
        # print(self.state_update)
        self.variance_update = np.dot(np.dot(self.F, self.variance), self.F.T) + self.Q
        print("variance update", self.variance_update[0][0], self.variance_update[1][1], self.variance_update[2][2], self.variance_update[9][9], self.variance_update[10][10])

    def update(self):
        # print("H * var", np.dot(self.H, self.variance_update))
        # self.K_t = np.dot( np.dot(self.variance_update, self.H.T), np.linalg.inv(np.dot( np.dot(self.H, self.variance_update), self.H.T)  + self.R) )
        # print("K_t", self.K_t[0][8], self.K_t[0][9])
        # print("CAPITAL S", np.dot( np.dot(self.H, self.variance_update), self.H.T)  + self.R)
        S = np.dot( np.dot(self.H, self.variance_update), self.H.T)  + self.R
        K_t = np.dot( np.dot(self.variance_update, self.H.T), np.linalg.inv(S) )
        print('state update after AT', self.state_update[0], self.state_update[1], self.state_update[2], self.state_update[9], self.state_update[10])
        print('z', self.z[6:8])
        print('estimated z', np.dot(self.H, self.state_update)[6:8])
        print('innovation', (self.z - np.dot(self.H, self.state_update))[6:8])
        # print(self.z - np.dot(self.H, self.state_update))
        print('kalman update term:', np.dot(K_t, (self.z - np.dot(self.H, self.state_update)))[0], np.dot(K_t, (self.z - np.dot(self.H, self.state_update)))[1], np.dot(K_t, (self.z - np.dot(self.H, self.state_update)))[2], np.dot(K_t, (self.z - np.dot(self.H, self.state_update)))[9], np.dot(K_t, (self.z - np.dot(self.H, self.state_update)))[10])
        # print(np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update))))
        self.state = self.state_update + np.dot(K_t, (self.z - np.dot(self.H, self.state_update)))
        # print("z-H.state", self.z - np.dot(self.H, self.state_update))
        self.variance = np.dot(np.identity(53) - np.dot(K_t, self.H), self.variance_update)
        # self.variance = np.dot(np.dot(np.identity(53) - np.dot(K_t, self.H), self.variance_update), (np.identity(53) - np.dot(K_t, self.H)).T) + np.dot(np.dot(K_t, self.R), K_t.T)
        # self.variance = self.variance_update - np.dot(K_t, np.dot(S, K_t.T))
        # print("variance", self.variance)
        self.H = self.H_core
        self.detected_tag = []


    


def main():
    rclpy.init()

    kf = KalmanFilter()
    pid = PIDcontroller(0.02, 0, 0.075)

    # move in a square path of 1.5m side
    # for _ in range(3):
    while(True):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.04
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        pid.publisher_.publish(twist_msg)

        time.sleep(delta_t)
        print("moving forward")

        input = np.array(([-calibration_x*twist_msg.linear.x/360], [calibration_y*twist_msg.linear.y/1.1], [calibration_ang*twist_msg.angular.z]))
        twist_msg.linear.y = 0.0
        kf.predict(input) # have to correct this input according to the kinematic model and rewrite

        
        pid.publisher_.publish(twist_msg)
        
        time.sleep(1.5)
        # print("HEREREREER")
        pid.get_measurement(kf)
        
        # print(kf.z[7], kf.z[8])
        
        kf.update() 
        
        print('state after update', kf.state[0], kf.state[1], kf.state[2], kf.state[9], kf.state[10])
        print()
        
        # if square side complete, break # TODO

            
if __name__ == '__main__':
    print("starting")
    main()