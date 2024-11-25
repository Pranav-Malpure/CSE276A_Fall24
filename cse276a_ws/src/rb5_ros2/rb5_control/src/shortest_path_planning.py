#!/usr/bin/env python3
import sys
#import rospy

# import rclpy 
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import PoseStamped
import numpy as np
import time 
import math
from collections import defaultdict
import pickle
import matplotlib.pyplot as plt
import visibility_graph as vg
from shapely.geometry import Polygon
from shapely.ops import linemerge

def main():
    g = vg.VisGraph()

    obstacle = [[vg.Point(3,3), vg.Point(5,3), vg.Point(5,5), vg.Point(3,5)], [vg.Point(6,2)], [vg.Point(2,6)]]
    g.build(obstacle)

    g_edges = g.visgraph.edges
    no_of_edges = (len(list(g_edges)))  
    for i in range(no_of_edges):
        plt.plot([list(g_edges)[i].p1.x, list(g_edges)[i].p2.x], [list(g_edges)[i].p1.y, list(g_edges)[i].p2.y], 'k')
    plt.title('Visibility_graph')
    plt.show()

if __name__ == '__main__':
    print("starting")
    
    main()

