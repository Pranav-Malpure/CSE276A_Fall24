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
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.new_pose_received = False
        self.subscription = self.create_subscription(
            PoseStamped,
            '/april_poses',
            self.pose_callback,
            10)     
        self.subscription
        # self.callback_data = {}
        self.callback_data = []
        self.position_history = []

    def pose_callback(self, msg):
        x = msg.pose.position.x
        z = msg.pose.position.z
        x_ang = msg.pose.orientation.x
        y_ang = msg.pose.orientation.y
        z_ang = msg.pose.orientation.z
        w_ang = msg.pose.orientation.w

        pitch = math.atan2(2 * (w_ang*y_ang - x_ang*z_ang), 1 - 2 * (y_ang*y_ang + z_ang*z_ang))
        pitch = (pitch + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)

        frame_id = msg.header.frame_id
        
        # self.callback_data[frame_id] = {'x': x, 'z': z, 'pitch': pitch}
        self.callback_data = [x, z, frame_id, pitch]


    def get_measurement(self, kf):
        rclpy.spin_once(self)
        while int(self.callback_data[2]) > 15:
            rclpy.spin_once(self)
        time.sleep(0.1)
        # print("callback data", self.callback_data)
        # theta = (kf.state_update[2])   # TODO: have to bound this in -pi to pi
        theta = kf.state_update[2][0]  # TODO: have to bound this in -pi to pi, and have to chose either this or above one
        # print("callback data", self.callback_data)
        # print("detected tag list ",kf.detected_tag)
        kf.variance_update[2][2] = 0 # ADDED: Variance update for angle is very small # TODO: can we do it at the initialization instead
        kf.z[(int(self.callback_data[2]) - 1)*2] = self.callback_data[0]
        kf.z[(int(self.callback_data[2]) - 1)*2 + 1] = self.callback_data[1]
        if kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] == 0 and kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] == 0:
            kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] = self.callback_data[0]*np.cos(theta) - self.callback_data[1]*np.sin(theta) + kf.state_update[0] # TODO: Add angle transformation of axes
            kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] = self.callback_data[0]*np.sin(theta) + self.callback_data[1]*np.cos(theta) + kf.state_update[1] # TODO: Add angle transformation of axes
            kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3] = 1e-4
            kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1] = 1e-4
            # print("variance update in get_measurement", kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3], kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1])

        if self.callback_data[2] not in kf.detected_tag:
            kf.detected_tag.append(self.callback_data[2])

        # kf.R = np.zeros((50, 50))
        for tag_list in kf.detected_tag:
            kf.H[(int(tag_list) - 1)*2][0] = -np.cos(theta)
            kf.H[(int(tag_list) - 1)*2][1] = -np.sin(theta)
            kf.H[(int(tag_list) - 1)*2 + 1][0] = np.sin(theta)
            kf.H[(int(tag_list) - 1)*2 + 1][1] = -np.cos(theta)
            kf.H[(int(tag_list) - 1)*2][(int(tag_list) - 1)*2 + 3] = np.cos(theta)
            kf.H[(int(tag_list) - 1)*2][(int(tag_list) - 1)*2 + 1 + 3] = np.sin(theta)

            kf.H[(int(tag_list) - 1)*2 + 1][(int(tag_list) - 1)*2 + 3] = -np.sin(theta)
            kf.H[(int(tag_list) - 1)*2 + 1][(int(tag_list) - 1)*2 + 1 + 3] = np.cos(theta)
            # kf.R[(int(tag_list) - 1)*2][(int(tag_list) - 1)*2] = 1e-2
            # kf.R[(int(tag_list) - 1)*2 + 1][(int(tag_list) - 1)*2 + 1] = 1e-2
            # kf.Q[(int(tag_list) - 1)*2 + 3][(int(tag_list) - 1)*2 + 3] = 1e-2
            # kf.Q[(int(tag_list) - 1)*2 + 3 + 1][(int(tag_list) - 1)*2 + 3 + 1] = 1e-2
        

        # kf.H[(int(self.callback_data[2]) - 1)*2][0] = -1
        # kf.H[(int(self.callback_data[2]) - 1)*2 + 1][1] = -1

        # #  _____________OLD CODE___________________
        # if self.callback_data[2] in kf.detected_tag:
        #     kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 3] = np.cos(theta)
        #     kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = -np.sin(theta)

        #     kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 3] = np.sin(theta)
        #     kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = np.cos(theta)
        #     # kf.H[int(self.callback_data[2])*2 - 1][int(self.callback_data[2])*2 - 1 + 3] = 1
        #     # kf.H[int(self.callback_data[2])*2][int(self.callback_data[2])*2 + 3] = 1
        # else:
        #     print("inside elsee")
        #     kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 3] = np.cos(theta)
        #     kf.H[(int(self.callback_data[2]) - 1)*2][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = -np.sin(theta)

        #     kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 3] = np.sin(theta)
        #     kf.H[(int(self.callback_data[2]) - 1)*2 + 1][(int(self.callback_data[2]) - 1)*2 + 1 + 3] = np.cos(theta)

            
        #     kf.z[(int(self.callback_data[2]) - 1)*2] = self.callback_data[0]
        #     kf.z[(int(self.callback_data[2]) - 1)*2 + 1] = self.callback_data[1]
        #     if kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] == 0 and kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] == 0:
        #         print('inside new tag')
        #         kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] = self.callback_data[0]*np.cos(theta) + self.callback_data[1]*np.sin(theta)  + kf.state_update[0] # TODO: Add angle transformation of axes
        #         kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] = -self.callback_data[0]*np.sin(theta) + self.callback_data[1]*np.cos(theta) + kf.state_update[1] # TODO: Add angle transformation of axes
        #         kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3] = 1e-2
        #         kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1] = 1e-2
        #         print("variance update in get_measurement", kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3], kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1])
        # print('state update after AT', kf.state_update[0], kf.state_update[1], kf.state_update[2], kf.state_update[10], kf.state_update[11])


    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = np.zeros(3)
        result[0] = targetState[0] - currentState[0]
        result[1] = targetState[1] - currentState[1]
        result[2] = targetState[2] - currentState[2]
        # result = targetState - currentState

        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 
    
    def update_sign(self, currentState, wp):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, wp)

        # P = np.array([self.Kp * e[0]/0.75, self.Kp * e[1], self.Kp * e[2]])
        # # self.I = np.array([self.I[0] + self.Ki * e[0] * self.timestep, self.I[1] + self.Ki * e[1] * self.timestep , self.I[2] + self.Ki * e[2] * self.timestep/2]) 
        # self.I = self.I + self.Ki * e * self.timestep
        # I = self.I
        # D = np.array([self.Kd * (e[0] - self.lastError[0]), self.Kd * (e[1] - self.lastError[1]), self.Kd * (e[2] - self.lastError[2])])
        # result = P + I + D

        # self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        # resultNorm = np.linalg.norm(result)
        # if(resultNorm > self.maximumValue):
        #     result = (result / resultNorm) * self.maximumValue
        #     self.I = 0.0
        result = [np.sign(e[0]), np.sign(e[1]), np.sign(e[2])]
        return result

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.target = np.array(state)

class KalmanFilter():
    def __init__(self):
        self.F = np.identity(53)
        self.G = np.zeros((53, 3)) 
        self.G[0] = [1,0,0]
        self.G[1] = [0,1,0] 
        self.G[2] = [0,0,1]
        self.G = delta_t*self.G
        # self.G = self.G*delta_t*r*0.25

        self.curpit = np.zeros(50)
        self.newpit = np.zeros(50)

        # self.variance = 1000*np.identity(53)
        self.variance = 100*np.identity(53)
        self.variance[0][0] = 0.0
        self.variance[1][1] = 0.0
        self.variance[2][2] = 0.0

        # self.Q = np.zeros((53, 53))
        self.Q = 1e-4*np.identity(53)
        self.Q[0][0] = 0.01
        self.Q[1][1] = 0.02
        self.Q[2][2] = 0.0

        self.K_t = np.zeros((53, 50))

        # self.S = np.zeros((50, 50))

        self.H = np.zeros((50, 53)) # H.s is actually where you think the april tag is, and z is actually where it is. it should be in robot frame
        # for i in range(50):
        #     if i % 2 == 0:
        #         self.H[i][0] = -1
        #     else:
        #         self.H[i][1] = -1 # subtract the x and y of the robot to get where the april tag can be
        
        self.z = np.zeros((50, 1))

        self.variance_update = np.zeros((53, 53))

        self.state = np.zeros((53, 1))
        # self.state[1] = 1/2
        # self.state[2] = 0

        self.state_update = np.zeros((53, 1))
        # self.state_update[1] = 1/2
        # self.state_update[2] = np.pi/2

        self.R = np.identity(50)*1e-4
        # self.R = np.zeros((50, 50))

        self.detected_tag = []

        self.states_track = []

    def predict(self, u):        
        self.state_update = np.dot(self.F, self.state) + np.dot(self.G,u)
        self.state_update[2][0] = (self.state_update[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
        # print("u", u)
        # print("G.u", np.dot(self.G, u))

        print("state update in PREDICT ", 'Robot', self.state_update[0], self.state_update[1], self.state_update[2], 'AT1', self.state_update[3], self.state_update[4], 'AT2', self.state_update[5], self.state_update[6], 'AT4', self.state_update[9], self.state_update[10], 'AT5', self.state_update[11], self.state_update[12], 'AT6', self.state_update[13], self.state_update[14], 'AT7', self.state_update[15], self.state_update[16], 'AT10', self.state_update[21], self.state_update[22], 'AT11', self.state_update[23], self.state_update[24])
        # print(self.state_update)
        self.variance_update = np.dot(np.dot(self.F, self.variance), self.F.T) + self.Q
        self.variance_update[2][2] = 0.0
        print("variance update ", 'Robot', self.variance_update[0][0], self.variance_update[1][1], self.variance_update[2][2], 'AT1', self.variance_update[3][3], self.variance_update[4][4], 'AT2', self.variance_update[5][5], self.variance_update[6][6], 'AT4', self.variance_update[9][9], self.variance_update[10][10], 'AT5', self.variance_update[11][11], self.variance_update[12][12], 'AT6', self.variance_update[13][13], self.variance_update[14][14], 'AT7', self.variance_update[15][15], self.variance_update[16][16], 'AT10', self.variance_update[21][21], self.variance_update[22][22], 'AT11', self.variance_update[23][23], self.variance_update[24][24])

    def update(self):
        # print("H * var", np.dot(self.H, self.variance_update))
        # self.K_t = np.dot( np.dot(self.variance_update, self.H.T), np.linalg.inv(np.dot( np.dot(self.H, self.variance_update), self.H.T)  + self.R) )
        # print("K_t", self.K_t[0][8], self.K_t[0][9])
        # print("CAPITAL S", np.dot( np.dot(self.H, self.variance_update), self.H.T)  + self.R)
        self.variance[2][2] = 0.0
        self.variance_update[2][2] = 0.0
        S = np.dot( np.dot(self.H, self.variance_update), self.H.T)  + self.R
        self.K_t = np.dot( np.dot(self.variance_update, self.H.T), np.linalg.inv(S))
        print('H CHECK', 'AT1', self.H[0][0], self.H[0][1], 'AT2', self.H[2][0], self.H[2][1], 'AT4', self.H[6][0], self.H[6][1], 'AT5', self.H[8][0], self.H[8][1], 'AT6', self.H[10][0], self.H[10][1], 'AT7', self.H[12][0], self.H[12][1], 'AT10', self.H[18][0], self.H[18][1], 'AT11', self.H[20][0], self.H[20][1])
        print('z CHECK', 'AT1', self.z[0][0], self.z[1][0], 'AT2', self.z[2][0], self.z[3][0], 'AT4', self.z[6][0], self.z[7][0], 'AT5', self.z[8][0], self.z[9][0], 'AT6', self.z[10][0], self.z[11][0], 'AT7', self.z[12][0], self.z[13][0], 'AT10', self.z[18][0], self.z[19][0], 'AT11', self.z[20][0], self.z[21][0])
        # print("K_t_0: ", self.K_t[0][6:8], self.K_t[0][0:2])
        # print("K_t_1: ", self.K_t[1][6:8], self.K_t[1][0:2])
        # print("K_t_9: ", self.K_t[9][6:8], self.K_t[1][0:2])
        # print("K_t_10: ", self.K_t[10][6:8], self.K_t[1][0:2])
        # print('state update after AT', self.state_update[0], self.state_update[1], self.state_update[2], self.state_update[3], self.state_update[4],self.state_update[9], self.state_update[10],self.state_update[11], self.state_update[12], self.state_update[13], self.state_update[14])
        print('Measured z ', 'AT1', self.z[0:2], 'AT2', self.z[2:4], 'AT4', self.z[6:8], 'AT5', self.z[8:10], 'AT6', self.z[10:12], 'AT7', self.z[12:14], 'AT10', self.z[18:20], 'AT11', self.z[20:22])
        print('Estimated z ', 'AT1', np.dot(self.H, self.state_update)[0:2], 'AT2', np.dot(self.H, self.state_update)[2:4], 'AT4', np.dot(self.H, self.state_update)[6:8], 'AT5', np.dot(self.H, self.state_update)[8:10], 'AT6', np.dot(self.H, self.state_update)[10:12], 'AT7', np.dot(self.H, self.state_update)[12:14], 'AT10', np.dot(self.H, self.state_update)[18:20], 'AT11', np.dot(self.H, self.state_update)[20:22])
        print('innovation ', 'AT1', (self.z - np.dot(self.H, self.state_update))[0:2], 'AT4', (self.z - np.dot(self.H, self.state_update))[6:8], 'AT5', (self.z - np.dot(self.H, self.state_update))[8:10], 'AT6', (self.z - np.dot(self.H, self.state_update))[10:12], 'AT11', (self.z - np.dot(self.H, self.state_update))[20:22])
        # print(self.z - np.dot(self.H, self.state_update))
        print('kalman update term:', 'Robot', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[0][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[1][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[2][0], 
              'AT1', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[3][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[4][0], 
              'AT2', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[5][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[6][0],
              'AT4', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[9][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[10][0], 
              'AT5', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[11][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[12][0], 
              'AT6', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[13][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[14][0], 
              'AT7', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[15][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[16][0], 
              'AT10', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[21][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[22][0], 
              'AT11', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[23][0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[24][0])
        # print(np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update))))
        self.state = self.state_update + np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))
        # print("z-H.state", self.z - np.dot(self.H, self.state_update))
        self.variance = np.dot(np.identity(53) - np.dot(self.K_t, self.H), self.variance_update)
        self.variance[2][2] = 0.0
        # self.variance = np.dot(np.dot(np.identity(53) - np.dot(K_t, self.H), self.variance_update), (np.identity(53) - np.dot(K_t, self.H)).T) + np.dot(np.dot(K_t, self.R), K_t.T)
        # self.variance = self.variance_update - np.dot(K_t, np.dot(S, K_t.T))
        # print("variance", self.variance)
        self.H = np.zeros((50, 53))
        self.z = np.zeros((50, 1))
        self.detected_tag = []


    


def main():
    # rclpy.init()

    # kf = KalmanFilter()
    # pid = PIDcontroller(0.02, 0, 0.075)

    # # move in a square path of 1.5m side
    # # for _ in range(3):
    # while(True):
    #     twist_msg = Twist()
    #     twist_msg.linear.x = 0.0
    #     twist_msg.linear.y = 0.04
    #     twist_msg.linear.z = 0.0
    #     twist_msg.angular.x = 0.0
    #     twist_msg.angular.y = 0.0
    #     twist_msg.angular.z = 0.0
    #     pid.publisher_.publish(twist_msg)

    #     time.sleep(delta_t)
    #     print("moving forward")

    #     input = np.array(([-calibration_x*twist_msg.linear.x/360], [calibration_y*twist_msg.linear.y/1.1], [calibration_ang*twist_msg.angular.z]))
    #     twist_msg.linear.y = 0.0
    #     kf.predict(input) # have to correct this input according to the kinematic model and rewrite

        
    #     pid.publisher_.publish(twist_msg)
        
    #     time.sleep(1.5)
    #     # print("HEREREREER")
    #     pid.get_measurement(kf)
        
    #     # print(kf.z[7], kf.z[8])
        
    #     kf.update() 
        
    #     print('state after update', kf.state[0], kf.state[1], kf.state[2], kf.state[9], kf.state[10])
    #     print()
        
    #     # if square side complete, break # TODO

        # NEW CLOSED LOOP code ________________________________________________________________________________________________________________

        rclpy.init()

        kf = KalmanFilter()
        pid = PIDcontroller(0.02, 0, 0.075)

        # waypoint = np.array([[0,0,0], [0, 1/2, 0], [0, 1/2, np.pi/2]])
        # waypoint = np.array([[0,1/2, 0],[0, 1/2, np.pi/2], [-1/2, 1/2, np.pi/2]])
        # waypoint = np.array([[1/2,1/2,-np.pi/4]])
        # waypoint = np.array([[0.0,1/2,0.0], [0.0, 1/2, np.pi/2], [-1/2, 1/2, np.pi/2], [-1/2, 1/2, -np.pi], [-1/2, 0, -np.pi], [-1/2, 0, -np.pi/2], [0,0, -np.pi/2], [0,0,0]])
        # square
        waypoint = np.array([[0, 0.4, 0], [0,0.8, 0], [0,0.8,np.pi/2], [-0.4,0.8,np.pi/2], [-0.8,0.8,np.pi/2], [-0.8,0.8,-np.pi], [-0.8,0.4,-np.pi], [-0.8,0,-np.pi], [-0.8,0,-np.pi/2], [-0.4,0,-np.pi/2], [0,0,-np.pi/2], [0,0,0], [0, 0.4, 0], [0,0.8, 0], [0,0.8,np.pi/2], [-0.4,0.8,np.pi/2], [-0.8,0.8,np.pi/2], [-0.8,0.8,-np.pi], [-0.8,0.4,-np.pi], [-0.8,0,-np.pi], [-0.8,0,-np.pi/2], [-0.4,0,-np.pi/2], [0,0,-np.pi/2], [0,0,0],]) # for square
        # octagon_side = 0.5/np.sqrt(2)
        # waypoint = np.array([
        #     [0.0, 0.0, -np.pi/8],
        #     [octagon_side*np.sin(np.pi/8), octagon_side*np.cos(np.pi/8), -np.pi/8],
        #     [octagon_side*np.sin(np.pi/8), octagon_side*np.cos(np.pi/8), np.pi/8],
        #     [0.0, 2*octagon_side*np.cos(np.pi/8), np.pi/8],
        #     [0.0, 2*octagon_side*np.cos(np.pi/8), 3*np.pi/8],
        #     [-octagon_side*np.cos(np.pi/8), 2*octagon_side*np.cos(np.pi/8) + octagon_side*np.sin(np.pi/8), 3*np.pi/8],
        #     [-octagon_side*np.cos(np.pi/8), 2*octagon_side*np.cos(np.pi/8) + octagon_side*np.sin(np.pi/8), 5*np.pi/8],
        #     [-2*octagon_side*np.cos(np.pi/8), 2*octagon_side*np.cos(np.pi/8), 5*np.pi/8],
        #     [-2*octagon_side*np.cos(np.pi/8), 2*octagon_side*np.cos(np.pi/8), 7*np.pi/8],
        #     [-2*octagon_side*np.cos(np.pi/8) - octagon_side*np.sin(np.pi/8), octagon_side*np.cos(np.pi/8), 7*np.pi/8],
        #     [-2*octagon_side*np.cos(np.pi/8) - octagon_side*np.sin(np.pi/8), octagon_side*np.cos(np.pi/8), -7*np.pi/8],
        #     [-2*octagon_side*np.cos(np.pi/8), 0.0, -7*np.pi/8],
        #     [-2*octagon_side*np.cos(np.pi/8), 0.0, -5*np.pi/8],
        #     [-octagon_side*np.cos(np.pi/8), -octagon_side*np.sin(np.pi/8), -5*np.pi/8],
        #     [-octagon_side*np.cos(np.pi/8), -octagon_side*np.sin(np.pi/8), -3*np.pi/8],
        #     [0.0, 0.0, -3*np.pi/8],
        #     [0.0, 0.0, 0]
        # ])


        # waypoint = np.array([[0, 1/2, np.pi/2]])
        seen_tags = set()
        for _ in range(25):
            rclpy.spin_once(pid)
            while int(pid.callback_data[2]) > 15:
                rclpy.spin_once(pid)
            time.sleep(0.1)
            frame_id, pitch = pid.callback_data[2], pid.callback_data[3]
            kf.curpit[int(frame_id) - 1] = pitch
            seen_tags.add(frame_id)
        time.sleep(0.2)

        for wp in waypoint:
            pid.setTarget(wp)
            print("move to way point", wp)
            # print("linalg: ", np.linalg.norm(pid.getError(kf.state[0:3], wp[0:3])))
            while rclpy.ok() and (np.sqrt((kf.state[0][0] - wp[0])**2 + (kf.state[1][0] - wp[1])**2 + (kf.state[2][0] - wp[2])**2) > 0.07):
                print()
                print("NEW OUTSIDE LOOP____________________________________________________________")
                twist_msg = Twist()
                robot_frame_state = [kf.state[0][0]*np.cos(kf.state[2][0]) + kf.state[1][0]*np.sin(kf.state[2][0]),
                                      -kf.state[0][0]*np.sin(kf.state[2][0]) + kf.state[1][0]*np.cos(kf.state[2][0]), kf.state[2][0]]

                wp_robot_frame = [wp[0]*np.cos(kf.state[2][0]) + wp[1]*np.sin(kf.state[2][0]),
                                    -wp[0]*np.sin(kf.state[2][0]) + wp[1]*np.cos(kf.state[2][0]), wp[2]]
                
                print("robot frame state", robot_frame_state)
                print("wp robot frame", wp_robot_frame)
                input_y_moved = np.array(([0], [0], [0]))
                # if (np.linalg.norm(pid.getError(kf.state[0:3], wp)[:2]) > 0.15):
                if abs(robot_frame_state[1] - wp_robot_frame[1]) > 0.15:
                    print('Y Big')
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.04*pid.update_sign(robot_frame_state, wp_robot_frame)[1]
                    twist_msg.linear.z = 0.0
                    twist_msg.angular.x = 0.0
                    twist_msg.angular.y = 0.0
                    twist_msg.angular.z = 0.0
                    pid.publisher_.publish(twist_msg)
                    time.sleep(delta_t)

                    theta_ = kf.state[2][0]
                    input_x = -np.sin(theta_)*calibration_y*twist_msg.linear.y/1.1
                    input_y = np.cos(theta_)*calibration_y*twist_msg.linear.y/1.1
                    input_y_moved = np.array(([input_x], [input_y], [calibration_ang*twist_msg.angular.z/1.45]))
                elif abs(robot_frame_state[1] - wp_robot_frame[1]) <= 0.15 and abs(robot_frame_state[1] - wp_robot_frame[1]) > 0.04:
                    print('Y Small')
                    theta_ = kf.state[2][0]
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.02*pid.update_sign(robot_frame_state, wp_robot_frame)[1]
                    twist_msg.linear.z = 0.0
                    twist_msg.angular.x = 0.0
                    twist_msg.angular.y = 0.0
                    twist_msg.angular.z = 0.0
                    pid.publisher_.publish(twist_msg)
                    time.sleep(delta_t)
                    input_x = -np.sin(theta_)*calibration_y*twist_msg.linear.y/1.3
                    input_y = np.cos(theta_)*calibration_y*twist_msg.linear.y/1.3
                    input_y_moved = np.array(([input_x], [input_y], [calibration_ang*twist_msg.angular.z/1.45]))
                print("moving forward")
                
                # Stop Car's
                twist_msg.linear.y = 0.0
                pid.publisher_.publish(twist_msg)
                time.sleep(0.4)

                input_x_moved = np.array(([0], [0], [0]))
                if abs(robot_frame_state[1] - wp_robot_frame[1]) < 0.05:
                    if abs(robot_frame_state[0] - wp_robot_frame[0]) > 0.04:
                        input_y_moved = np.array(([0], [0], [0]))
                        print('inside x correction')
                        theta_ = kf.state[2][0]
                        twist_msg.linear.x = 0.05*pid.update_sign(robot_frame_state, wp_robot_frame)[0]
                        twist_msg.linear.y = 0.0
                        twist_msg.linear.z = 0.0
                        twist_msg.angular.x = 0.0
                        twist_msg.angular.y = 0.0
                        twist_msg.angular.z = 0.0
                        pid.publisher_.publish(twist_msg)
                        time.sleep(delta_t)
                        input_x = np.cos(theta_)*calibration_x*twist_msg.linear.x/9
                        input_y = np.sin(theta_)*calibration_x*twist_msg.linear.x/9
                        input_x_moved = np.array(([input_x], [input_y], [calibration_ang*twist_msg.angular.z/1.45]))
                        twist_msg.linear.x = 0.0
                        pid.publisher_.publish(twist_msg)

                print('X and Y velocites calculated')
                input = input_x_moved + input_y_moved
                # Stop Car's x
                twist_msg.linear.x = 0.0
                
                pid.publisher_.publish(twist_msg)
                time.sleep(0.5)
                # Predict state in open loop
                kf.predict(input)

                # Measure april tag detection    
                it_seen = set()
                for _ in range(8):   
                    rclpy.spin_once(pid)  
                    while int(pid.callback_data[2]) > 15:
                        rclpy.spin_once(pid)
                    time.sleep(0.1)      
                    frame_id, pitch = pid.callback_data[2], pid.callback_data[3]
                    kf.newpit[int(frame_id) - 1] = pitch
                    it_seen.add(frame_id)

                # ang_rot = 0.0
                # for tag in seen_tags.intersection(it_seen):
                #     ang_rot += (kf.newpit[int(tag) - 1] - kf.curpit[int(tag) - 1])
                # ang_rot = ang_rot / len(seen_tags.intersection(it_seen))

                # kf.state_update[2][0] = kf.state_update[2][0] + ang_rot
                # kf.state_update[2][0] = kf.state_update[2][0] + ang_rot
                # kf.state[2][0] = (kf.state[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
                # kf.state_update[2][0] = (kf.state_update[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
                
                min_dist = 1e5
                min_tag = 100
                for tag in seen_tags.intersection(it_seen):
                    d = np.sqrt((kf.state[0][0] - kf.state[2*(int(tag)-1)+3][0])**2 + (kf.state[1][0] - kf.state[2*(int(tag)-1)+1+3][0])**2)
                    if d < min_dist:
                        min_dist = d
                        min_tag = tag               

                kf.state[2][0] += (kf.newpit[int(min_tag) - 1] - kf.curpit[int(min_tag) - 1])
                kf.state[2][0] = (kf.state[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
                kf.state_update[2][0] += (kf.newpit[int(min_tag) - 1] - kf.curpit[int(min_tag) - 1])
                kf.state_update[2][0] = (kf.state_update[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)

                kf.curpit = kf.newpit.copy()
                seen_tags = it_seen.copy()
                time.sleep(0.1)

                for i in range(8):
                    pid.get_measurement(kf)
                print('DETECTED TAGS THIS ITR', kf.detected_tag)
            
                # Reconcile measured and predicted measurements
                kf.update() 
                print('DETECTED TAGS CLEARED OR NOT?', kf.detected_tag)

                print("______STATES(L)_________", 'Robot', kf.state[0], kf.state[1], kf.state[2], 'AT1', kf.state[3], kf.state[4], 'AT2', kf.state[5], kf.state[6], 'AT4', kf.state[9], kf.state[10], 'AT5', kf.state[11], kf.state[12], 'AT6', kf.state[13], kf.state[14], 'AT7', kf.state[15], kf.state[16], 'AT10', kf.state[21], kf.state[22], 'AT11', kf.state[23], kf.state[24])

                # kf.states_track.append([kf.state[0][0], kf.state[1][0], kf.state[2][0]])  
                kf.states_track.append(kf.state)   

                print()
                # print("error", np.linalg.norm(pid.getError(kf.state[0:3][0], wp[0:3])[:2]))
                print("error", np.sqrt((kf.state[0][0] - wp[0])**2 + (kf.state[1][0] - wp[1])**2))
                print("WAYPOINT:  ", wp)

                print()

                if (np.sqrt((kf.state[0][0] - wp[0])**2 + (kf.state[1][0] - wp[1])**2)) < 0.06:
                    print('inside angle regime')

                    while rclpy.ok() and (abs(kf.state[2][0] - wp[2])) > 0.05:
                        robot_frame_state = [kf.state[0][0]*np.cos(kf.state[2][0]) + kf.state[1][0]*np.sin(kf.state[2][0]),
                                             -kf.state[0][0]*np.sin(kf.state[2][0]) + kf.state[1][0]*np.cos(kf.state[2][0]), kf.state[2][0]]

                        wp_robot_frame = [wp[0]*np.cos(kf.state[2][0]) + wp[1]*np.sin(kf.state[2][0]),
                                          -wp[0]*np.sin(kf.state[2][0]) + wp[1]*np.cos(kf.state[2][0]), wp[2]]
                        print("ANGLE ERROR: ", abs(kf.state[2][0] - wp[2]))
                        # rotating (1 movment = x rad)
                        twist_msg = Twist()
                        twist_msg.linear.x = 0.0
                        twist_msg.linear.y = 0.0
                        twist_msg.linear.z = 0.0
                        twist_msg.angular.x = 0.0
                        twist_msg.angular.y = 0.0
                        twist_msg.angular.z = 0.1*pid.update_sign(robot_frame_state, wp_robot_frame)[2]
                        pid.publisher_.publish(twist_msg)
                        time.sleep(2*delta_t)
                        print("rotating")

                        # input = np.array(([-calibration_x*twist_msg.linear.x/360], [calibration_y*twist_msg.linear.y/1.1], [calibration_ang*twist_msg.angular.z/1.45])) # TODO: have to calibrate the angle
                        input = np.array(([-calibration_x*twist_msg.linear.x/360], [calibration_y*twist_msg.linear.y/1.1], [0.0]))
                        # Stop Car
                        twist_msg.angular.z = 0.0
                        pid.publisher_.publish(twist_msg)
                        time.sleep(0.5)
                        # Predict state in open loop
                        # kf.predict(input)

                        # Measure april tag detection  
                        it_seen = set()
                        for _ in range(25):      
                            rclpy.spin_once(pid)   
                            while int(pid.callback_data[2]) > 15:
                                rclpy.spin_once(pid)    
                            frame_id, pitch = pid.callback_data[2], pid.callback_data[3]
                            kf.newpit[int(frame_id) - 1] = pitch
                            it_seen.add(frame_id)

                        # ang_rot = 0.0
                        # for tag in seen_tags.intersection(it_seen):
                        #     ang_rot += (kf.newpit[int(tag) - 1] - kf.curpit[int(tag) - 1])
                        # ang_rot = ang_rot / len(seen_tags.intersection(it_seen))

                        # kf.state_update[2][0] = kf.state_update[2][0] + ang_rot
                        # kf.state_update[2][0] = kf.state_update[2][0] + ang_rot
                        # kf.state[2][0] = (kf.state[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
                        # kf.state_update[2][0] = (kf.state_update[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)                        
                        
                        min_dist = 1e5
                        min_tag = 100
                        for tag in seen_tags.intersection(it_seen):
                            d = np.sqrt((kf.state[0][0] - kf.state[2*(int(tag)-1)+3][0])**2 + (kf.state[1][0] - kf.state[2*(int(tag)-1)+1+3][0])**2)
                            if d < min_dist:
                                min_dist = d
                                min_tag = tag
                        
                        if min_tag != 100:
                            kf.state[2][0] += (kf.newpit[int(min_tag) - 1] - kf.curpit[int(min_tag) - 1])
                            kf.state_update[2][0] += (kf.newpit[int(min_tag) - 1] - kf.curpit[int(min_tag) - 1])
                            pass
                        else:
                            # TODO: kf.state[2] = Record the rotation estimated from open loop
                            pass
                        kf.state[2][0] = (kf.state[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
                        kf.state_update[2][0] = (kf.state_update[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)

                        kf.curpit = kf.newpit.copy()
                        seen_tags = it_seen.copy()
                        time.sleep(0.1)

                        # for _ in range(25):           
                        #     pid.get_measurement(kf)

                        # Reconcile measured and predicted measurements
                        # kf.update() 

                        print("_____STATES(A)_____: ", 'Robot', kf.state[0], kf.state[1], kf.state[2], 'AT1', kf.state[3], kf.state[4], 'AT2', kf.state[5], kf.state[6], 'AT4', kf.state[9], kf.state[10], 'AT5', kf.state[11], kf.state[12], 'AT6', kf.state[13], kf.state[14], 'AT7', kf.state[15], kf.state[16], 'AT10', kf.state[21], kf.state[22], 'AT11', kf.state[23], kf.state[24])


                print("ERROR AT END: ", np.sqrt((kf.state[0][0] - wp[0])**2 + (kf.state[1][0] - wp[1])**2))

            if wp == [0,0,0]:
                with open('final_state_square_1.pkl', 'wb') as file:    # Save state to .pkl
                    pickle.dump(kf.state, file)
                with open('states_track_square_1.pkl', 'wb') as file:   # Save robot trajectory
                    pickle.dump(kf.states_track, file)
                with open('final_variance_square_1.pkl', 'wb') as file:    # Save variance to .pkl
                    pickle.dump(kf.variance, file)       
       
        with open('final_state_square_2.pkl', 'wb') as file:    # Save state to .pkl
            pickle.dump(kf.state, file)
        with open('states_track_square_2.pkl', 'wb') as file:   # Save robot trajectory
            pickle.dump(kf.states_track, file)
        with open('final_variance_square_2.pkl', 'wb') as file:    # Save variance to .pkl
            pickle.dump(kf.variance, file)       


if __name__ == '__main__':
    print("starting")
    main()