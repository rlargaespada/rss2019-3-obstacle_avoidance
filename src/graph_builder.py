#!/usr/bin/env python2
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseWithCovarianceStamped, PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
import matplotlib.pyplot as plt
import graph

class Graph_Build:
    """
    Builds a graph object from an image map and saves to a pickle file.
    """
    def __init__(self):
        """
        start = [x, y, theta]
        goal = [x, y]
        """
        # topic params
        self.START_TOPIC = rospy.get_param("~start_topic")
        self.GOAL_TOPIC = rospy.get_param("~goal_topic")
        self.MAP_TOPIC = rospy.get_param("~map_topic")
        #self.PATH_TOPIC = rospy.get_param("~path_topic")

        self.start_pose = [0, 0, 0] # x, y, theta
        self.goal_pose = [0, 0, 0] # x, y, theta

        #map creation parameters
        self.map_res = rospy.get_param("~map_res")
        self.map_name = rospy.get_param("~map")
        self.x_bounds = rospy.get_param('x_bounds') #tuple (x_min, y_min) or None
        self.y_bounds = rospy.get_param('y_bounds') #tuple (x_min, y_min) or None
        self.origin_x_offset = rospy.get_param("~origin_x_offset")
        self.origin_y_offset = rospy.get_param("~origin_y_offset")
        self.origin = None

        self.goal_list = []

        #Initialize visualization variables
        self.buff_factor = rospy.get_param("~buff_map")

        self.map_loaded = False
        self.map_graph = None
        self.map_file = 'map_file'

        # initialize publishers and subscribers
        # self.particle_cloud_publisher = rospy.Publisher(self.PARTICLE_CLOUD_TOPIC, PointCloud, queue_size=10)
        # self.path_publisher = rospy.Publisher(self.PATH_TOPIC, PointCloud, queue_size=10)


        # rospy.Subscriber(self.START_TOPIC, PoseWithCovarianceStamped, self.set_start)
        # rospy.Subscriber(self.GOAL_TOPIC, PoseStamped, self.set_goal)

        rospy.Subscriber(
                self.MAP_TOPIC,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    # def real_world_to_occ(self, coord, resolution, origin):
    #     '''converts coordinates from the "real world" frame to the occupancy grid frame'''
    #     x = int((coord[0]+origin.position.x)/resolution)
    #     y = int((coord[1]+origin.position.y)/resolution)
    #     return (x,y)

    # def occ_to_real_world(self, coord, resolution, origin):
    #     x = coord[0]*resolution - origin.position.x
    #     y = coord[1]*resolution - origin.position.y
    #     return (x,y)

    def set_start(self, start_pose):
        """
        Gets starting pose from rviz pose estimate marker.
        """
        x, y = start_pose.pose.pose.position.x, start_pose.pose.pose.position.y
        theta = 2*np.arctan(start_pose.pose.pose.orientation.z/start_pose.pose.pose.orientation.w)

        self.start_pose = [round(x, 1), round(y, 1), theta]

    def set_goal(self, goal_pose):
        """
        Gets goal pose from rviz nav goal marker.
        """
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y
        theta = 2*np.arctan(goal_pose.pose.pose.orientation.z/goal_pose.pose.pose.orientation.w)
        #goal = [round(x*2, 0)/2., round(y*2, 0)/2., 0]
        goal = [round(x, 1), round(y, 1), theta]

        self.goal_pose = goal
        self.goal_list.append(goal)

    def map_callback(self, map_msg):
        if self.map_loaded: return

        # print "Loading map:", rospy.get_param("~map"), "..."
        # print "Start and Goal intialized:"
        # print "Start: ", self.start_pose
        #print "Goal: ", self.goal_pose
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)
        map_ = np.clip(map_, 0, 1)
        self.map = np.reshape(map_, (map_msg.info.height, map_msg.info.width)).T
        # self.map = np.flip(self.map, axis=0)
        self.map_copy = np.copy(self.map)
        #Beef up the edges
        for i in range(self.map_copy.shape[0]): #change range to appropriate coordiates of map
            for j in range(self.map_copy.shape[1]): #change range to appropriate coordiates of map
                if self.map_copy[i, j] != 0:
                    self.map[i-self.buff_factor: i+self.buff_factor, j-self.buff_factor: j+self.buff_factor] = 1.0
                    
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        self.origin = (origin_p.x+self.origin_x_offset, origin_p.y+self.origin_y_offset, origin_o[2])

        #map is an array of zeros and ones, convert into graph
        self.map_graph = graph.Graph()
        self.resolution = map_msg.info.resolution
        print(map_msg.info.resolution, map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        self.map_graph.build_map(self.map, self.map_name, self.resolution, self.origin, self.x_bounds, self.y_bounds)

        self.map_loaded = True
        print("yay, we built the graph!")


if __name__ == "__main__":
    rospy.init_node("astar")
    builder = Graph_Build()
    rospy.spin()