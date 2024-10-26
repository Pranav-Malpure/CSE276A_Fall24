#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mpi_control import MegaPiController
import numpy as np
import time


# using a scaled down version of the waypoints
waypoints = [
[0,0,0],
[-1/2,0,0],
[-1/2,1/2,1.57],
[-2/2,1/2,0],
[-2/2,2/2,-1.57],
[-1/2,1/2,-0.78],
[0,0,0]
]


initial_time = 0
current_pose = [0, 0, 0]
sleep_time = 1

# Constants used:

Kv = 2 # this is the factor which gets multiplied with linear velocity to give the number to pass to the carStraight function, has to be callibrated
Ktheta = 1 # this is the factor which gets multiplied with angular velocity to give the number to pass to the carRotate function, has to be callibrated
threshold_distance = 0.1 # callibrated depending on how fine you want the car to follow the path
lx = 0.0675 #Horizontal distance between wheel axis and vertical axis of the car
ly = 0.057 # Vertical distance between the wheel axis and horizontal axis of the car
rw = 0.03 #Radius of the wheel
angular_vel_rotate = 1.04
linear_vel_straight = 0.13
v_epsilon = 2
factor = 9.24


class MegaPiControllerNode(Node):
    def __init__(self, verbose=False, debug=False):
        super().__init__('megapi_controller_node')
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)

    # This is a ROS2 Foxy template for MegaPi controller


    def get_under_range(self, theta): #Range is [-pi, pi)
        theta %= 6.28; 
        theta = (theta + 6.28) % 6.28;  
        if theta > 3.14:
            theta -= 6.28
        if theta == 3.14:
            theta = -3.14
        return theta

    def calc_diff_theta(self, theta_target, theta): # output is in radians
        diff = theta_target - theta
        if diff >= 3.14:
            diff -= 6.28
        elif diff < -3.14:
            diff += 6.28
        return diff


    def follow_waypoints(self, waypoints):
        global current_pose, Kv, sleep_time, Ktheta, threshold_distance, angular_vel_rotate, linear_vel_straight, factor
        print("initial hello from function follow waypoints")
        waypoints_index = 0
        linear_distance = np.sqrt((waypoints[0][0] - current_pose[0])**2 + (waypoints[0][1] - current_pose[1])**2) #initializing the linear distance
        print("hello1")
        try:
            while True:
                print("hello2")                
                print ("linear dist",linear_distance)
                print ("THRESHOLD DIST", threshold_distance)
                time.sleep(1)
                if linear_distance < threshold_distance:
                    print('orientation delta', self.calc_diff_theta(waypoints[waypoints_index][2], current_pose[2]))
                    print("current_waypoint for orientation", waypoints[waypoints_index])
                    if abs(self.calc_diff_theta(waypoints[waypoints_index][2], current_pose[2])) > 0.12: # 0.12 radians is 7degrees
                        # calculate omega that will make the robot rotate towards the waypoint
                        angle_to_be_moved = self.calc_diff_theta(waypoints[waypoints_index][2], current_pose[2]) # output is in radians
                        time_in_seconds = abs(angle_to_be_moved)*7/(2*3.14) + 0.2
                        print("angle time in seconds", time_in_seconds)
                        print('angle to be moved', angle_to_be_moved)
                        vx = 0
                        vy=0
                        omegaz = angular_vel_rotate 
                        omega1 = (1 / rw) * (vx - vy - (lx+ly)*omegaz)
                        omega2 = (1 / rw) * (vx + vy + (lx+ly)*omegaz)
                        omega3 = (1 / rw) * (vx + vy - (lx+ly)*omegaz)
                        omega4 = (1 / rw) * (vx - vy + (lx+ly)*omegaz)
                        if angle_to_be_moved >= 0:
                            self.mpi_ctrl.setFourMotors(int(omega1*factor), int(omega2*factor), int(omega3*factor), int(omega4*factor))

                        else:
                            self.mpi_ctrl.setFourMotors(int(omega1*factor), int(omega2*factor), int(omega3*factor), int(omega4*factor))

                        time.sleep(time_in_seconds)
                        self.mpi_ctrl.carStop()
                        time.sleep(2)
                        #this will align the robot to the waypoint
                   
                    current_pose[2] = waypoints[waypoints_index][2] # update current pose 
                    print('current_pose angle after orientation', current_pose[2])

                    waypoints_index = waypoints_index + 1
                    if waypoints_index == len(waypoints):
                        self.mpi_ctrl.carStop()
                        print("DESTINATION REACHED")
                        break 
                        
                    theta_target = np.arctan2(waypoints[waypoints_index][1] - current_pose[1], waypoints[waypoints_index][0] - current_pose[0])
                    print('theta_target', theta_target)
                    print('\nalignment', self.calc_diff_theta(theta_target, current_pose[2]))
                    if abs(self.calc_diff_theta(theta_target, current_pose[2])) > 0.12:
                        print("current_waypoint for alignment", waypoints[waypoints_index])
                        angle_to_be_moved = self.calc_diff_theta(theta_target, current_pose[2]) # output is in radians
                        time_in_seconds = abs(angle_to_be_moved)*7/(2*3.14) + 0.2
                        print("angle time in seconds", time_in_seconds)
                        print('angle to be moved', angle_to_be_moved)
                        print("SEcond loop")

                        vx = 0
                        vy=0
                        omegaz = angular_vel_rotate 
                        omega1 = (1 / rw) * (vx - vy - (lx+ly)*omegaz)
                        omega2 = (1 / rw) * (vx + vy + (lx+ly)*omegaz)
                        omega3 = (1 / rw) * (vx + vy - (lx+ly)*omegaz)
                        omega4 = (1 / rw) * (vx - vy + (lx+ly)*omegaz)
                        print(int(omega1*factor), int(omega2*factor), int(omega3*factor), int(omega4*factor))
                        if angle_to_be_moved >= 0:
                            self.mpi_ctrl.setFourMotors(int(omega1*factor), int(omega2*factor), int(omega3*factor), int(omega4*factor))

                        else:
                            self.mpi_ctrl.setFourMotors(int(omega1*factor), int(omega2*factor), int(omega3*factor), int(omega4*factor))

                        time.sleep(time_in_seconds)
                        self.mpi_ctrl.carStop()
                        time.sleep(0.5)
                        #this will align the robot to the waypoint
                   
                    current_pose[2] = theta_target 
                    print('current_pose angle after alignment', current_pose[2])
                    
                linear_distance = np.sqrt((waypoints[waypoints_index][0] - current_pose[0])**2 + (waypoints[waypoints_index][1] - current_pose[1])**2)
                
                time_in_seconds = linear_distance*7.05/1 + 0.3
                vx = linear_vel_straight
                vy = 0
                omegaz = 0
                omega1 = (1 / rw) * (vx - vy - (lx+ly)*omegaz)
                omega2 = (1 / rw) * (vx + vy + (lx+ly)*omegaz)
                omega3 = (1 / rw) * (vx + vy - (lx+ly)*omegaz)
                omega4 = (1 / rw) * (vx - vy + (lx+ly)*omegaz)
                self.mpi_ctrl.setFourMotors(-int(omega1*factor), int(omega2*factor), int(omega3*factor), -int(omega4*factor))

                time.sleep(time_in_seconds)
                self.mpi_ctrl.carStop()
                time.sleep(0.5)
                current_pose[0] = waypoints[waypoints_index][0]
                current_pose[1] = waypoints[waypoints_index][1]
                print('current pos of robot', current_pose)
                print('current waypoint for linear distance', waypoints[waypoints_index])
                linear_distance = np.sqrt((waypoints[waypoints_index][0] - current_pose[0])**2 + (waypoints[waypoints_index][1] - current_pose[1])**2)

                time.sleep(2)
                print("ONE WAYPOINT COMPLETED")
        
        except KeyboardInterrupt:
            print("Control-c pressed")
            self.mpi_ctrl.carStop()
            self.mpi_ctrl.close()




if __name__ == "__main__":
    # waypoints, initial_time, initial_pose
    initial_time = time.time()
    
    print('before init')
    rclpy.init()
    print("hello from main 1")
    mpi_ctrl_node = MegaPiControllerNode()
    print("hello from main 2")
    # Start ROS2 node
    mpi_ctrl_node.follow_waypoints(waypoints)
    
    rclpy.spin(mpi_ctrl_node) # Spin for until
    # Destroy node and shutdown when done. (Optional, as node would be cleaned up by garbage collection)
    mpi_ctrl_node.destroy_node()
    rclpy.shutdown()