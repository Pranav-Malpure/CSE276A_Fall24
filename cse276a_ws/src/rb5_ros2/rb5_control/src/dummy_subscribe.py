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

callback_data = defaultdict(dict)

def pose_callback(msg):
    x = msg.pose.position.x
    z = msg.pose.position.z
    x_ang = msg.pose.orientation.x
    y_ang = msg.pose.orientation.y
    z_ang = msg.pose.orientation.z
    w_ang = msg.pose.orientation.w

    pitch = math.atan2(2 * (w_ang*y_ang - x_ang*z_ang), 1 - 2 * (y_ang*y_ang + z_ang*z_ang))
    pitch = (pitch + math.pi) % (2 * math.pi) - math.pi # scale to range [-pi, pi)

    frame_id = msg.header.frame_id
    
    callback_data[frame_id] = {'x': x, 'z': z, 'pitch': pitch}

def subscriber():
    rclpy.init()
    node = rclpy.create_node('dummy_subscriber')
    subscription = rclpy.create_subscription(
            PoseStamped,
            '/april_poses',
            pose_callback(),
            10)


if __name__ == '__main__':
    subscriber()
    for i in range(10):
        rclpy.spin_once()
    print(callback_data)
