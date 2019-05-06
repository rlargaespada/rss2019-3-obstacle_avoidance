#!/usr/bin/env python2

import numpy as np
import math
import rospy
import scipy
import dubin
from sklearn.metrics import mean_squared_error
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32, Point, PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

class Dubins_Avoidance(object):
    def __init(self):

        self.START_TOPIC = rospy.get_param("~start_topic")
        self.GOAL_TOPIC = rospy.get_param("~goal_topic")
        self.POSE_ESTIM_TOPIC = rospy.get_param("~pose_estim_topic")
        self.LASERSCAN_TOPIC = rospy.get_param("~scan_topic")
        self.SCAN_STEP = rospy.get_param('~scan_step')
        self.ORIGIN = np.array(rospy.get_param("~origin"))
        self.TURNING_RADIUS = rospy.get_param("~turning_radius")
        self.DUBINS_PATH_STEP = rospy.get_param("~dubins_path_step")
        self.PARTICLE_CLOUD_TOPIC = rospy.get_param("~cloud_topic")
        self.PATH_TOPIC = rospy.get_param("~path_topic")

        self.pose_estim = None #[x, y, theta], map frame
        self.start = None #[x, y, theta], map frame
        self.goal = None #[x, y, theta], map frame
        self.paths = None

        self.in_scan_cb = False
        self.in_pose_cb = False
        self.visualize_all_paths = False



        rospy.Subscriber(self.POSE_ESTIM_TOPIC, Point32, self.pose_estim_callback)
        rospy.Subscriber(self.LASERSCAN_TOPIC, LaserScan, self.scan_callback)
        rospy.Subscriber(self.START_TOPIC, PoseWithCovarianceStamped, self.set_start)
        rospy.Subscriber(self.GOAL_TOPIC, PoseStamped, self.set_goal)

        self.particle_cloud_publisher = rospy.Publisher(self.PARTICLE_CLOUD_TOPIC, PointCloud, queue_size=10)
        self.path_publisher = rospy.Publisher(self.PATH_TOPIC, PointCloud, queue_size=10)

    def set_start(self, start_pose):
        x, y = start_pose.pose.pose.position.x, start_pose.pose.pose.position.y
        theta = 2*np.arctan(start_pose.pose.pose.orientation.z/start_pose.pose.pose.orientation.w)
        self.start_pose = [round(x, 1), round(y, 1), theta]

    def set_goal(self, goal_pose):
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y
        theta = 2*np.arctan(goal_pose.pose.orientation.z/goal_pose.pose.orientation.w)
        self.goal_pose = [round(x, 1), round(y, 1), theta]

    def pose_estim_callback(self, pose):
        if self.in_scan_cb: return
        self.in_pose_cb = True
        self.pose_estim = pose
        self.in_pose_cb = False

    def scan_callback(self, scan_data):
        if self.in_pose_cb: return
        if self.start == None or self.goal == None: return
        self.in_scan_cb = True

        self.paths = self.get_paths(scan_data)

        if self.visualize_all_paths: self.visualize_paths()

        self.choose_path()
        self.visualize_chosen_path()
        self.publish_path()

        self.in_scan_cb = False

    def get_paths(self, scan_data):
        paths = []
        step = self.SCAN_STEP/scan_data.angle_increment

        dubin_goals = set()
        for i in range(0, len(scan_data.ranges), step):
            r = scan_data.ranges[i]
            theta = scan_data.angle_min*scan_data.angle_increment*i
            x = r*math.cos(theta)
            y = r*math.sin(theta)
            map_frame_xy = np.array([x, y]) - self.ORIGIN[:2] #need to make sure this is right
            goal_theta = self.goal_pose[2] #fix so that this is the angle between current heading and goal point
            dubin_goals.add((map_frame_xy[0], map_frame_xy[1], goal_theta))

        for goal in dubin_goals:
            p = dubins.shortest_path(tuple(self.pose_estim), goal, self.TURNING_RADIUS)
            path = p.sample_many(self.DUBINS_PATH_STEP)
            paths.append(path)

        return paths

    def visualize_paths(self):
        pass

    def choose_path(self):
        pass

    def visualize_chosen_path(self):
        pass

    def publish_path(self):
        pass
        