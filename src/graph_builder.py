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
        TODO
        """
        # topic params
        self.MAP_TOPIC = rospy.get_param("~map_topic")

        #map creation parameters
        self.map_name = rospy.get_param("~map")
        self.resolution = None
        self.x_bounds = rospy.get_param('x_bounds') #tuple (x_min, y_min) or None
        self.y_bounds = rospy.get_param('y_bounds') #tuple (x_min, y_min) or None
        self.origin_x_offset = rospy.get_param("~origin_x_offset")
        self.origin_y_offset = rospy.get_param("~origin_y_offset")
        self.origin = None
        self.buff_factor = rospy.get_param("~buff_map")

        self.map_loaded = False
        self.map_graph = None

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
        for i in range(self.map_copy.shape[0]): 
            for j in range(self.map_copy.shape[1]):
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
    rospy.init_node("graph_builder")
    builder = Graph_Build()
    rospy.spin()