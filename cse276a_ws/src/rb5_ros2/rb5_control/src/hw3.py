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

        pitch = math.atan2(2 * (w_ang*y_ang - x_ang*z_ang), 1 - 2 * (y_ang*y_ang + z_ang*z_ang))
        pitch = (pitch + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)

        frame_id = msg.header.frame_id
        
        self.callback_data = [x, z, frame_id, pitch]

    def get_measurement(self, kf):
        rclpy.spin_once(self)
        # print("callback data", self.callback_data)
        # theta = (kf.state_update[2])   # TODO: have to bound this in -pi to pi
        theta = kf.state_update[2][0]  # TODO: have to bound this in -pi to pi, and have to chose either this or above one
        # print("callback data", self.callback_data)
        # print("detected tag list ",kf.detected_tag)

        kf.variance_update[2][2] = 1e-2 # ADDED: Variance update for angle is very small # TODO: can we do it at the initialization instead
        kf.z[(int(self.callback_data[2]) - 1)*2] = self.callback_data[0]
        kf.z[(int(self.callback_data[2]) - 1)*2 + 1] = self.callback_data[1]
        if kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] == 0 and kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] == 0:
            print('inside new tag')
            kf.state_update[(int(self.callback_data[2]) - 1)*2 + 3] = self.callback_data[0]*np.cos(theta) + self.callback_data[1]*np.sin(theta)  + kf.state_update[0] # TODO: Add angle transformation of axes
            kf.state_update[(int(self.callback_data[2]) - 1)*2 + 1 + 3] = -self.callback_data[0]*np.sin(theta) + self.callback_data[1]*np.cos(theta) + kf.state_update[1] # TODO: Add angle transformation of axes
            kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3] = 1e-2
            kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1] = 1e-2
            print("variance update in get_measurement", kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3][(int(self.callback_data[2]) - 1)*2 + 3], kf.variance_update[(int(self.callback_data[2]) - 1)*2 + 3 + 1][(int(self.callback_data[2]) - 1)*2 + 3 + 1])

        if self.callback_data[2] not in kf.detected_tag:
            kf.detected_tag.append(self.callback_data[2])


        for tag_list in kf.detected_tag:
            kf.H[(int(tag_list) - 1)*2][0] = -1
            kf.H[(int(tag_list) - 1)*2 + 1][1] = -1
            kf.H[(int(tag_list) - 1)*2][(int(tag_list) - 1)*2 + 3] = np.cos(theta)
            kf.H[(int(tag_list) - 1)*2][(int(tag_list) - 1)*2 + 1 + 3] = -np.sin(theta)

            kf.H[(int(tag_list) - 1)*2 + 1][(int(tag_list) - 1)*2 + 3] = np.sin(theta)
            kf.H[(int(tag_list) - 1)*2 + 1][(int(tag_list) - 1)*2 + 1 + 3] = np.cos(theta)

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
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 
    
    def update_sign(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

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
        self.variance = 1000*np.identity(53)
        self.variance[0][0] = 0
        self.variance[1][1] = 0
        self.variance[2][2] = 0

        # self.Q = np.zeros((53, 53))
        self.Q = np.identity(53)
        self.Q[0][0] = 0.01
        self.Q[1][1] = 0.02
        self.Q[2][2] = 0.0

        self.K_t = np.zeros((53, 50))

        # self.S = np.zeros((50, 50))

        self.H_core = np.zeros((50, 53))
        # for i in range(50):
        #     if i % 2 == 0:
        #         self.H_core[i][0] = -1
        #     else:
        #         self.H_core[i][1] = -1 

        self.H = np.zeros((50, 53)) # H.s is actually where you think the april tag is, and z is actually where it is. it should be in robot frame
        # for i in range(50):
        #     if i % 2 == 0:
        #         self.H[i][0] = -1
        #     else:
        #         self.H[i][1] = -1 # subtract the x and y of the robot to get where the april tag can be
        
        self.z = np.zeros((50, 1))

        self.variance_update = np.zeros((53, 53))

        self.state = np.zeros((53, 1))

        self.state_update = np.zeros((53, 1))

        self.R = np.identity(50)*1e-2

        self.detected_tag = []

        self.states_track = []

    def predict(self, u):        
        self.state_update = np.dot(self.F, self.state) + np.dot(self.G,u)
        self.state_update[2][0] = (self.state_update[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
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
        self.K_t = np.dot( np.dot(self.variance_update, self.H.T), np.linalg.inv(S) )
        print("K_t_0: ", self.K_t[0][6:8], self.K_t[0][0:2])
        print("K_t_1: ", self.K_t[1][6:8], self.K_t[1][0:2])
        print("K_t_9: ", self.K_t[9][6:8], self.K_t[1][0:2])
        print("K_t_10: ", self.K_t[10][6:8], self.K_t[1][0:2])
        print('state update after AT', self.state_update[0], self.state_update[1], self.state_update[2], self.state_update[9], self.state_update[10])
        print('z', self.z[6:8], self.z[0:2])
        print('estimated z', np.dot(self.H, self.state_update)[6:8], np.dot(self.H, self.state_update)[0:2])
        print('innovation', (self.z - np.dot(self.H, self.state_update))[6:8], (self.z - np.dot(self.H, self.state_update))[0:2])
        # print(self.z - np.dot(self.H, self.state_update))
        print('kalman update term:', np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[0], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[1], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[2], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[9], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[10], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[3], np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))[4])
        # print(np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update))))
        self.state = self.state_update + np.dot(self.K_t, (self.z - np.dot(self.H, self.state_update)))
        # print("z-H.state", self.z - np.dot(self.H, self.state_update))
        self.variance = np.dot(np.identity(53) - np.dot(self.K_t, self.H), self.variance_update)
        # self.variance = np.dot(np.dot(np.identity(53) - np.dot(K_t, self.H), self.variance_update), (np.identity(53) - np.dot(K_t, self.H)).T) + np.dot(np.dot(K_t, self.R), K_t.T)
        # self.variance = self.variance_update - np.dot(K_t, np.dot(S, K_t.T))
        # print("variance", self.variance)
        self.H = self.H_core
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

        waypoint = np.array([[0,1/2,0], [0, 1/2, np.pi/2], [-1/2, 1/2, np.pi/2]])
        rclpy.spin_once(pid)
        seen_tags = []
        for _ in range(25):
            frame_id, pitch = pid.callback_data[2], pid.callback_data[3]
            kf.curpit[int(frame_id) - 1] = pitch
            seen_tags.append(frame_id)
        time.sleep(3)
        
        for wp in waypoint:
            pid.setTarget(wp)
            print("move to way point", wp)
            # print("linalg: ", np.linalg.norm(pid.getError(kf.state[0:3], wp[0:3])))
            while rclpy.ok() and (np.linalg.norm(pid.getError(kf.state[0:3], wp[0:3])) > 0.05):
                twist_msg = Twist()
                if (np.linalg.norm(pid.getError(kf.state[0:3][0], wp)[:2]) > 0.15):
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.04*pid.update_sign(kf.state[0:3][0])[1]
                    twist_msg.linear.z = 0.0
                    twist_msg.angular.x = 0.0
                    twist_msg.angular.y = 0.0
                    twist_msg.angular.z = 0.0
                    pid.publisher_.publish(twist_msg)
                    time.sleep(delta_t)
                else:
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.02*pid.update_sign(kf.state[0:3])[1]
                    twist_msg.linear.z = 0.0
                    twist_msg.angular.x = 0.0
                    twist_msg.angular.y = 0.0
                    twist_msg.angular.z = 0.0
                    pid.publisher_.publish(twist_msg/2)
                    time.sleep(delta_t)
                print("moving forward")

                input = np.array(([-calibration_x*twist_msg.linear.x/360], [calibration_y*twist_msg.linear.y/1.1], [calibration_ang*twist_msg.angular.z/1.45]))
                # Stop Car
                twist_msg.linear.y = 0.0
                pid.publisher_.publish(twist_msg)
                time.sleep(1.5)
                # Predict state in open loop
                kf.predict(input)
                # Measure april tag detection    
                it_seen = []
                for _ in range(25):           
                    pid.get_measurement(kf)
                    frame_id, pitch = pid.callback_data[2], pid.callback_data[3]
                    kf.newpit[int(frame_id) - 1] = pitch
                    it_seen.append(frame_id)
                # Reconcile measured and predicted measurements
                kf.update() 
                
                ang_rot = 0
                for tag in seen_tags:
                    if tag in it_seen:
                        ang_rot += (kf.newpit[int(tag) - 1] - kf.curpit[int(tag) - 1])
                ang_rot /= len(it_seen)
                kf.state[2][0] += ang_rot
                kf.state[2][0] = (kf.state[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
                kf.curpit = kf.newpit.copy()
                seen_tags = it_seen[:]

                print(kf.state[0], kf.state[1], kf.state[2], kf.state[3], kf.state[4], kf.state[9], kf.state[10]) 

                kf.states_track.append([kf.state[0][0], kf.state[1][0], kf.state[2][0]])     

                print()
                print("error", np.linalg.norm(pid.getError(kf.state[0:3], wp)[:2]))
                print()

                if (np.linalg.norm(pid.getError(kf.state[0:3], wp[0:3])[:2]) < 0.1):
                    print('inside angle regime')
                    seen_tags = []
                    for _ in range(25):
                        frame_id, pitch = pid.callback_data[2], pid.callback_data[3]
                        kf.curpit[int(frame_id) - 1] = pitch
                        seen_tags.append(frame_id)

                    time.sleep(1)
                    while rclpy.ok() and abs(pid.getError(kf.state[0:3], wp)[2]) > 0.03:
                        # rotating (1 movment = x rad)
                        twist_msg = Twist()
                       
                        twist_msg.linear.x = 0.0
                        twist_msg.linear.y = 0.0
                        twist_msg.linear.z = 0.0
                        twist_msg.angular.x = 0.0
                        twist_msg.angular.y = 0.0
                        twist_msg.angular.z = 0.1*pid.update_sign(kf.state[0:3])[2]
                        pid.publisher_.publish(twist_msg)
                        time.sleep(2*delta_t)
                        print("rotating")

                        input = np.array(([-calibration_x*twist_msg.linear.x/360], [calibration_y*twist_msg.linear.y/5], [calibration_ang*twist_msg.angular.z/1.45])) # TODO: have to calibrate the angle
                        # Stop Car
                        twist_msg.angular.z = 0.0
                        pid.publisher_.publish(twist_msg)
                        time.sleep(1.5)
                        # Predict state in open loop
                        kf.predict(input)
                        # Measure april tag detection  
                        it_seen = []
                        for _ in range(25):           
                            pid.get_measurement(kf)
                            frame_id, pitch = pid.callback_data[2], pid.callback_data[3]
                            kf.newpit[int(frame_id) - 1] = pitch
                            it_seen.append(frame_id)
                        # Reconcile measured and predicted measurements
                        kf.update() 
                        ang_rot = 0
                        for tag in seen_tags:
                            if tag in it_seen:
                                ang_rot += (kf.newpit[int(tag) - 1] - kf.curpit[int(tag) - 1])
                        ang_rot /= len(it_seen)
                        if ang_rot != 0:
                            kf.state[2][0] += ang_rot
                            kf.state[2][0] = (kf.state[2][0] + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)
                        else:
                            # TODO: kf.state[2] = Record the rotation estimated from open loop
                            pass
                        kf.curpit = kf.newpit.copy()
                        seen_tags = it_seen[:]

                        print(kf.state[0], kf.state[1], kf.state[2], kf.state[3], kf.state[4], kf.state[9], kf.state[10])

        # with open('final_state.pkl', 'wb') as file:    # Save state to .pkl
        #     pickle.dump(kf.state, file)
        # with open('states_track.pkl', 'wb') as file:   # Save robot trajectory
        #     pickle.dump(kf.states_track, file)
        # with open('final_variance.pkl', 'wb') as file:    # Save variance to .pkl
        #     pickle.dump(kf.variance, file)       



if __name__ == '__main__':
    print("starting")
    main()