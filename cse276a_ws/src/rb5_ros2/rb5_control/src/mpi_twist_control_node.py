#!/usr/bin/env python3
""" MegaPi Controller ROS2 Wrapper"""
#import rospy
import rclpy # replaces rospy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from mpi_control import MegaPiController
import numpy as np

"""
This file include code that control the robot motors
+,+,+,+ left
-,+,+,- straight
-,+,-,+ anticlockwise rotation

140, 215, 230
"""

class MegaPiControllerNode(Node):
    def __init__(self, verbose=True, debug=False):
        super().__init__('megapi_controller_node')
        self.mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=verbose)
        self.r = 0.025 # radius of the wheel
        self.lx = 0.055 # half of the distance between front wheel and back wheel
        self.ly = 0.07 # half of the distance between left wheel and right wheel
        self.calibration_y = 60 #140
        self.calibration_x = 200#215
        self.calibration_ang = 60 #230
        self.subscription = self.create_subscription(Twist, '/twist', self.twist_callback, 10)
        self.subscription

    def twist_callback(self, twist_cmd):
        # note below we have changed the order of the axis because the robot's motors have a misaligned axis as well
        desired_twist = np.array([[-self.calibration_x*twist_cmd.linear.x], [self.calibration_y*twist_cmd.linear.y], [self.calibration_ang*twist_cmd.angular.z]])
        # calculate the jacobian matrix
        jacobian_matrix = np.array([[1, -1, -(self.lx + self.ly)],
                                     [1, 1, (self.lx + self.ly)],
                                     [1, 1, -(self.lx + self.ly)],
                                     [1, -1, (self.lx + self.ly)]]) / self.r
        # calculate the desired wheel velocity
        result = np.dot(jacobian_matrix, desired_twist)
        
        # def map_value(value, in_min, in_max, out_min, out_max):
        #     # Linear mapping function
        #     return out_min + (float(value - in_min) / float(in_max - in_min)) * (out_max - out_min)

        # # Define the input and output ranges
        # input_min, input_max = 0, 220  #input range
        # positive_output_min, positive_output_max = 35, 100
        # negative_output_min, negative_output_max = -100, -35

        # # Map each motor value based on its sign
        # def map_motor(value):
        #     if value >= 0:
        #         return int(map_value(value, input_min, input_max, positive_output_min, positive_output_max))
        #     else:
        #         return int(map_value(value, -input_max, -input_min, negative_output_min, negative_output_max))

        # motor1 = int(map_motor(result[0][0]))
        # motor2 = int(map_motor(result[1][0]))
        # motor3 = int(map_motor(result[2][0]))
        # motor4 = int(map_motor(result[3][0]))


        # send command to each wheel
        # self.mpi_ctrl.setFourMotors(motor1, motor2, motor3, motor4)
        self.mpi_ctrl.setFourMotors(int(result[0][0]+ np.sign(result[0][0])*35), int(result[1][0]+np.sign(result[1][0])*35), int(result[2][0]+np.sign(result[2][0])*35), int(result[3][0]+np.sign(result[3][0])*35))


        

if __name__ == "__main__":
    rclpy.init()
    mpi_ctrl_node = MegaPiControllerNode()
    #rospy.init_node('megapi_controller')
    #rospy.Subscriber('/twist', Twist, mpi_ctrl_node.twist_callback, queue_size=1) 

    
    rclpy.spin(mpi_ctrl_node) # Spin for until shutdown

    # Destroy node and shutdown when done. (Optional, as node would be cleaned up by garbage collection)
    mpi_ctrl_node.destroy_node()
    rclpy.shutdown()
