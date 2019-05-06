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
        self.full_view_ang = rospy.get_param("~full_view_ang")  #The +/- angle from the goal from which we take chunks
        self.discard_dist = rospy.get_param("~discard_dist")    #The +/- number of scans we consider for discarding a chunk due to a close scan
        self.discard_size = rospy.get_param("~discard_size")    #The distance threshold considered to discard a chunk within the scans from discard_dist
        self.chunk_size = rospy.get_param("~chunk_size")        #The +/- number of scans we consider for a chunk to be scored
        self.chunk_spacing = rospy.get_param("~chunk_spacing")  #The indicies between chunks we take
        self.VELOCITY = rospy.get_param("~velocity")            #Velocity of the racecar
        self.k_dist = rospy.get_param("~min_dist_weight")       #Weight given to the minimum distance in the clearance portion of score function
        self.k_goal = rospy.get_param("~goal_ang_weight")       #Weight given to the magnitude of the angle between a chunk and the goal
        self.k_pose = rospy.get_param("~past_ang_weight")       #Weight given to the magnitude of the angle between a chunk and the last angle chosen
        self.out = AckermannDriveStamped()
        # self.create_message(self.VELOCITY)
        self.pose = np.zeros(3)     #Pose of robot
        self.goal = np.zeros(3)     #Goal point
        self.scan = np.array([])      
        self.heading = 0            #Direction the wheels face
        self.min_ang = -2*pi/3.     #Minimum angle of the scan
        self.heading_pub = rospy.Publisher(self.HEADING_TOPIC, Marker, queue_size=10)                   #Publishes heading as a marker [CURRENTLY INACTIVE]
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)              #Pubhlishes the drive command
        rospy.Subscriber(self.POSE_TOPIC,PoseStamped,self.pose_callback,queue_size=10)  #Gets pose from localization 
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)                     #Gets laserscan
        rospy.Subscriber(self.GOAL_TOPIC, PoseStamped, self.set_goal)                   #Gets the new goal position


    def callback(self, scan):
        '''
        gets laserscan and sets important values
        '''
        self.scan = np.array(scan.ranges)
        self.angs_list = self.a_trans(scan)
        self.min_ang = scan.angle_min
        self.ang_inc = scan.angle_increment

    def set_goal(self, goal_pose):
        """
        Gets starting pose from rviz pose estimate marker.
        """
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y
        self.goal = np.array([x, y, 0])

    def pose_callback(self, pose):
        '''
        input: Pose from localization as a PoseStamped
        output: None, publishes a message of the best heading for the robot
        '''
        self.pose = np.array([pose.pose.position.x,pose.pose.position.y,2*np.arctan(pose.pose.orientation.z/pose.pose.orientation.w)]) 
        # self.pose = np.array([pose.x,pose.y,pose.z]) #sets global position variable
        #Set angle to goal
        self.goal_ang = np.arctan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0]) - self.pose[2]
        #Get index in angles for the minimum and maximum angles from the angle list (also corrosponds to self.scan)
        min_idx = self.to_usable_angle(self.goal_ang - self.full_view_ang)
        max_idx = self.to_usable_angle(self.goal_ang + self.full_view_ang)
        #Initialize max score and group
        max_score = -float("inf")       #Score for closeness correct direction
        max_group = 0                   #Gropu with best heading
        print("____BEGIN__________________")
        #Iterate through the chunks in the scan
        for group in range(min_idx, max_idx, self.chunk_spacing):
            #Loop thorough chunk_size scan sections of laserscan
            chunk_dists = self.scan[group - self.chunk_size: group + self.chunk_size]           #Get chunk distances
            chunk_discard = self.scan[group - self.discard_size: group + self.discard_size]     #Get safety region for the chunk
            ang_from_goal = abs(self.angs_list[group] - self.goal_ang)                          #Get absolute value of angle from goal
            ang_from_odom = abs(self.angs_list[group])                                          #Get absolute value of angle from current pose angle
            #Get the score for this group
            temp_score = self.get_score(chunk_dists, chunk_discard, ang_from_goal, ang_from_odom)
            print(self.angs_list[group], temp_score, chunk_dists.min(), ang_from_goal, ang_from_odom)
            #If this group is better than previous best group, set this group as new max_group
            if temp_score >= max_score:
                max_score = temp_score
                max_group = group
        print("MAX:")
        print(self.angs_list[max_group], max_score)
        #Determine heading
        self.determine_heading(max_group)
        #Send steering mesage
        self.create_message(self.VELOCITY, self.heading)
        self.pub.publish(self.out)


    def determine_heading(self, max_group):
        '''
        input: the max group from the scoring
        output: none, but sets the heading
        #TODO: Probably should be something with ackermann steering. Currently just points wheels in direction of the max group found.
        '''
        self.heading = max(-.34, min(.34, self.angs_list[max_group]))
        

    
    def get_score(self, dists, dists_discard, ang_from_goal, ang_from_odom):
        '''
        input: dists: numpy array of the distances for the "clearance" section that we will take for scoring
               dists_discard: the dists in the range of values that can cause the chunk to be discarded outright
               ang_from_goal: absolute value of angle to the goal
               ang_from_odom: absolute value of angle from current pose angle
        output: The score of a group given the input values
        '''
        #If any distance in the discard is less than the threshold, return -inf
        if dists_discard.min() < self.discard_dist:
            print("discarded")
            return -float("inf")
        #Return the weighted score of each relevant input
        return dists.min()*self.k_dist - ang_from_goal*self.k_goal - ang_from_odom*self.k_goal


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

    def to_usable_angle(self, theta):
        '''
        input: theta: angle in racecar's frame
        output: index corresponding to the closest theta from self.ang_list
        '''
        angle_to_min = theta - self.min_ang
        index = int(angle_to_min/self.ang_inc)
        return max(0, min(index, len(self.angs_list)))


    def a_trans(self,data):
        '''
        input: data: laserscan message
        output: list of what each angle in the laser scan represents
        '''
        #returns [list] of angles within range
        amin = data.angle_min #min angle [rad]
        amax = data.angle_max #max angle [rad]
        ainc = data.angle_increment #min angle increment [rad]
        angs_list = [amin]
        angs_dict = {}
        for i in range(len(data.ranges)):
            angs_list.append(angs_list[i]+ainc)
        return angs_list


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
    rospy.init_node('obstacle_avoidance')
    avoidance = ObstacleAvoidance()
    rospy.spin()