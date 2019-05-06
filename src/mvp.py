#!/usr/bin/env python2

import numpy as np
import math
import rospy
import scipy
from sklearn.metrics import mean_squared_error
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32, Point, PoseStamped, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped

pi = np.pi

class ObstacleAvoidance:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT

    def __init__(self):
        # TODO:
        self.POSE_TOPIC = rospy.get_param("~local_topic")
        self.HEADING_TOPIC = rospy.get_param("~heading_marker")
        self.SCAN_TOPIC = rospy.get_param("~scan_topic")
        self.DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.GOAL_TOPIC = rospy.get_param("~goal_topic")
        self.dodge_ang = rospy.get_param("~dodge_ang")
        self.dodge_dist = rospy.get_param("~dodge_dist")
        self.chunk_size = rospy.get_param("~chunk_size")
        self.VELOCITY = rospy.get_param("~velocity")
        self.out = AckermannDriveStamped()
        # self.create_message(self.VELOCITY)
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        #Contains the parameters of the last colision
        self.pose_sub = rospy.Subscriber(self.POSE_TOPIC,PoseStamped,self.pose_callback,queue_size=10)
        self.pose = np.zeros(3)
        self.goal = np.zeros(3)
        self.scan = np.array([])
        self.goal_ang = 0
        self.heading = 0 
        self.overide_bounds = [-self.dodge_ang, self.dodge_ang]
        self.last_overide = None
        self.heading_pub = rospy.Publisher(self.HEADING_TOPIC, Marker, queue_size=10)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        rospy.Subscriber(self.GOAL_TOPIC, PoseStamped, self.set_goal)


    def callback(self, scan):
        self.scan = np.array(scan.ranges)
        self.center = len(scan.ranges)/2
        self.angs = self.a_trans(scan)

    def set_goal(self, goal_pose):
        """
        Gets starting pose from rviz pose estimate marker.
        """
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y

        self.goal = np.array([x, y, 0])

    def pose_callback(self, pose):
        self.pose = np.array([pose.pose.position.x,pose.pose.position.y,2*np.arctan(pose.pose.orientation.z/pose.pose.orientation.w)]) 
        # self.pose = np.array([pose.x,pose.y,pose.z]) #sets global position variable
        #Set angle to goal
        self.goal_ang = np.arctan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0]) - self.pose[2]
        max_score = float("inf")        #Score for closeness correct direction
        overide_dist = self.dodge_dist  #Distance to beat to divert car
        overide = False                 #Has been diverted?
        overide_ang = 0                 #Angle that causes overide
        max_group = 0                   #Gropu with best heading
        for group in range(0, len(self.scan), self.chunk_size):
            #Loop thorough 10 scan sections of laserscan
            avg_dist = np.average(self.scan[group: group + self.chunk_size]) 
            x, y = self.pol_to_cart(avg_dist, self.angs[group + self.chunk_size/2])
            if avg_dist < overide_dist and y < .3 and x < self.dodge_dist:
                #If average distance is triggered, set distance and angle
                overide_dist = avg_dist
                overide = True
                overide_ang = self.angs[group]
            ang_off = self.angs[group] - self.goal_ang
            ang_corrected = min(abs(ang_off - 2*pi), abs(ang_off + 2*pi), abs(ang_off))
            score = ang_corrected
            #Check if this beats last heading
            if score < max_score:
                max_score = score
                max_group = group
        # print(score)
        self.determine_heading(overide, max_group, overide_ang)
        self.create_PointCloud()
        self.create_message(self.VELOCITY, self.heading)
        self.pub.publish(self.out)


    def pol_to_cart(self, r, th):
        #Convert a polar point to cartesian point
        return r*np.cos(th), r*np.sin(th)



    def determine_heading(self, overide, max_group, overide_ang):
        if overide:
            # print("DODGE")
            if (overide_ang < 0 or self.last_overide == "left") and self.last_overide!= "right":
                self.heading = .34
                self.last_overide = "left"
            else:
                self.heading = -.34
                self.last_overide
                self.last_overide = "right"
        else:
            # print("FOLLOW")
            self.heading = self.angs[max_group]
            self.last_overide = None



    def create_message(self, v, ang):
        """create optput AckermannDriveStamped mssage
        """
        self.out.header.stamp = rospy.Time.now()
        self.out.header.frame_id = "1"
        self.out.drive.steering_angle = ang
        self.out.drive.steering_angle_velocity = 0
        self.out.drive.speed = v
        self.out.drive.acceleration = 0
        self.out.drive.jerk = 0


    def a_trans(self,data):
	#returns [list] of angles within range
	amin = data.angle_min #min angle [rad]
	amax = data.angle_max #max angle [rad]
	ainc = data.angle_increment #min angle increment [rad]
	angs = [amin]
	for i in range(len(data.ranges)):
		angs.append(angs[i]+ainc)
        return angs


    def pol_to_cart(self, r, th):
        #Convert a polar point to cartesian point
        return r*np.cos(th), r*np.sin(th)

    def create_PointCloud(self):

        #arrow marker for current pose
        heading = Marker()
        heading.header.frame_id = "/map"
        heading.header.stamp = rospy.Time.now()
        heading.ns = "heading_marker"
        heading.id = 0
        heading.type = heading.ARROW
        heading.action = heading.ADD

        #start point and end point
        heading.points = [Point(), Point()]
        #start point
        heading.points[0].x = self.pose[0]
        heading.points[0].y = self.pose[1]
        heading.points[0].z = 0
        #end point
        heading.points[1].x = np.cos(self.heading) + self.pose[0]
        heading.points[1].y = np.sin(self.heading) + self.pose[1]
        heading.points[1].z = 0

        heading.scale.x = 0.2
        heading.scale.y = 0.4

        heading.color.a = 1.0
        heading.color.g = 1.0
        self.heading_pub.publish(heading)



if __name__ == "__main__":
    rospy.init_node('mvp')
    avoidance = ObstacleAvoidance()
    rospy.spin()