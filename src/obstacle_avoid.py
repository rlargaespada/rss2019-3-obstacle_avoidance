#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import PolygonStamped, Point32, PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point
from sensor_msgs.msg import PointCloud, LaserScan
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import tf
import matplotlib.pyplot as plt
import graph
import search
import math

class Map_Builder(object):
    def __init__(self):
        
        self.START_TOPIC = rospy.get_param("~start_topic")
        self.GOAL_TOPIC = rospy.get_param("~goal_topic")
        self.SCAN_TOPIC = rospy.get_param("~scan_topic")
        self.MAP_TOPIC = rospy.get_param("~map_topic")
        self.MAP_FILE = rospy.get_param("~map_file") #stata_basement0.1m.p
        self.SCAN_HIT_INC = rospy.get_param("~scan_hit_inc")
        self.OBSTACLE_THRESHOLD = rospy.get_param("~obstacle_threshold")
        self.POSE_ESTIM_TOPIC = rospy.get_param("~pos_estim_topic")

        self.origin_x_offset = rospy.get_param("~origin_x_offset")
        self.origin_y_offset = rospy.get_param("~origin_y_offset")
        self.map_res = rospy.get_param("~map_res")

        self.start_pose = [0, 0, 0] # x, y, theta
        self.goal_pose = [0, 0, 0] # x, y, theta
        self.current_pose =  [0,0,0]
        self.map_loaded = False
        self.map_graph = None
        self.graph_node_vals = dict()
        self.grid = None
        self.origin = None

        self.load_map()

        rospy.Subscriber(self.START_TOPIC, PoseWithCovarianceStamped, self.set_start)
        rospy.Subscriber(self.GOAL_TOPIC, PoseStamped, self.set_goal)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)
        rospy.Subscriber(self.POSE_ESTIM_TOPIC, Point32, self.pose_estim_callback)
        rospy.Subscriber(self.MAP_TOPIC, OccupancyGrid, self.occ_callback, queue_size=1)

    def set_start(self, start_pose):
        x, y = start_pose.pose.pose.position.x, start_pose.pose.pose.position.y
        theta = 2*np.arctan(start_pose.pose.pose.orientation.z/start_pose.pose.pose.orientation.w)
        self.start_pose = [round(x, 1), round(y, 1), theta]

    def set_goal(self, goal_pose):
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y
        theta = 2*np.arctan(goal_pose.pose.orientation.z/goal_pose.pose.orientation.w)
        self.goal_pose = [round(x, 1), round(y, 1), theta]


    def load_map(self):
        if self.map_loaded: pass

        self.map_graph = graph.Graph()
        self.map_graph.load_map(self.MAP_FILE)
        for n in self.map_graph.nodes:
            self.graph_node_vals[n] = 0
        self.map_loaded = True

    def pose_estim_callback(self, pose):
        #add timing gate
        self.current_pose = [pose.x, pose.y, pose.z]


    def occ_callback(self, occ_grid):
    	map_ = np.array(occ_grid.data, np.double)
    	map_ = np.clip(map_, 0, 1)
    	self.grid = np.reshape(map_, (occ_grid.info.height, occ_grid.info.width)).T
    	origin_p = occ_grid.info.origin.position
        origin_o = occ_grid.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        self.origin = (origin_p.x+self.origin_x_offset, origin_p.y+self.origin_y_offset, origin_o[2])


    def scan_callback(self, ls):
        if not self.map_loaded: pass
        #add timing gate

        #from laserscan data, find likely obstacles and correlate these to locations relative to the car
        relative_obstacle_positions = []
        for alpha in range(len(ls.ranges)):
            if ls.range_min <= ls.ranges[alpha] <= ls.range_max: #better way?
                r = ls.ranges[alpha]
                angle = ls.angle_min + ls.angle_increment*alpha
                x = r*math.cos(angle)
                y = r*math.sin(angle)
                relative_obstacle_positions.append((x, y))

        #correlate these to nodes within the graph (map frame)
        #probably use einsum?
        #fir each position in r_o_p, convert to global frame and round to nearest .1 meter
        obstacle_nodes = []

        #increase the value of these nodes by self.SCAN_HIT_INC
        for n in obstacle_nodes:
            self.graph_node_vals[n] += self.SCAN_HIT_INC

        #if any node has a value greater than a self.OBSTACLE_THRESHOLD, remove it from the graph
        for n in self.graph_node_vals:
            if self.graph_node_vals[n] > self.OBSTACLE_THRESHOLD:
                self.map_graph.remove_node(n)

        #publish relevant data