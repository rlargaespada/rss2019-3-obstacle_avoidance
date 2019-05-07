#!/usr/bin/env python2

import numpy as np
import rospy
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
        self.POSE_TOPIC = rospy.get_param("~local_topic")
        self.HEADING_TOPIC = rospy.get_param("~heading_marker")
        self.SCAN_TOPIC = rospy.get_param("~scan_topic")
        self.DRIVE_TOPIC = rospy.get_param("~drive_topic")
        self.GOAL_TOPIC = rospy.get_param("~goal_topic")
        self.SAFETY_TOPIC = rospy.get_param("~safety_topic")

        self.full_view_ang = rospy.get_param("~full_view_ang")  #The +/- angle from the goal from which we take sects
        self.safety_dist = rospy.get_param("~safety_dist")    #The +/- number of scans we consider for safetying a sect due to a close scan
        self.safety_size = rospy.get_param("~safety_size")    #The distance threshold considered to safety a sect within the scans from safety_dist
        self.safety_thresh = rospy.get_param("~safety_thresh")
        self.clearance_dist = rospy.get_param("~clearance_dist")
        self.sect_size = rospy.get_param("~sect_size")        #The +/- number of scans we consider for a sect to be scored
        self.sect_spacing = rospy.get_param("~sect_increment")  #The indicies between sects we take
        self.max_velocity = rospy.get_param("~max_velocity")            #velocity of the racecar
        self.k_dist = rospy.get_param("~clearance_weight")       # Weight given to the minimum distance in the clearance portion of score function
        self.k_goal = rospy.get_param("~goal_weight")       #Weight given to the magnitude of the angle between a sect and the goal
        self.k_turn = rospy.get_param("~turn_weight")       #Weight given to the magnitude of the angle between a sect and the last angle chosen
        self.drive_msg = AckermannDriveStamped()

        self.pose = np.zeros(3)     # Pose of robot
        self.velocity = self.max_velocity # current speed of the robot
        self.map_density = 1        # 1 being low density, 3 being high density
        self.goal = np.zeros(3)     # Goal point
        self.goal_region = {"xmin": 0, "xmax": 0, "ymin": 0, "ymax": 0}
        self.goal_size = 0.5
        self.in_goal = False
        self.scan = np.array([])
        self.heading = 0            # Direction the wheels face
        self.min_ang = -2*pi/3.     # Minimum angle of the scan

        # pubs and subs
        self.heading_pub = rospy.Publisher(self.HEADING_TOPIC, Marker, queue_size=10) # Publishes heading as a marker
        self.safety_pub = rospy.Publisher(self.SAFETY_TOPIC, Marker, queue_size=10)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)              #Pubhlishes the drive command
        rospy.Subscriber(self.POSE_TOPIC,PoseStamped, self.pose_cb, queue_size=10)  #Gets pose from localization
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_cb)                     #Gets laserscan
        rospy.Subscriber(self.GOAL_TOPIC, PoseStamped, self.goal_cb)                   #Gets the new goal position

    def scan_cb(self, scan):
        """
        Gets laserscan and sets important values
        """
        self.scan = np.array(scan.ranges)
        self.angs_list = self.a_trans(scan)
        self.min_ang = scan.angle_min
        self.ang_inc = scan.angle_increment
        self.ang_inc_deg = self.ang_inc*180./pi

    def goal_cb(self, goal_pose):
        """
        Gets goal pose from rviz pose estimate marker.
        """
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y
        self.goal = np.array([x, y, 0])

        self.goal_region["xmin"] = x-self.goal_size
        self.goal_region["xmax"] = x+self.goal_size
        self.goal_region["ymin"] = y-self.goal_size
        self.goal_region["ymax"] = y+self.goal_size

    def check_goal(self):
        """
        Sets self.in_goal to True if we are have reached our goal.
        """
        if self.goal_region["xmin"] < self.pose[0] < self.goal_region["xmax"]:
            if self.goal_region["ymin"] < self.pose[1] < self.goal_region["ymax"]:
                print "Reached goal!"
                self.in_goal = True

    def pose_cb(self, pose):
        """
        Input: Pose from localization as a PoseStamped
        Output: None, publishes a message of the best heading for the robot
        """
        self.pose = np.array([pose.pose.position.x,pose.pose.position.y,2*np.arctan(pose.pose.orientation.z/pose.pose.orientation.w)])
        # self.pose = np.array([pose.x,pose.y,pose.z]) #sets global position variable
        #Set angle to goal
        self.goal_ang = np.arctan2(self.goal[1] - self.pose[1], self.goal[0] - self.pose[0]) - self.pose[2]
        #Get index in angles for the minimum and maximum angles from the angle list (also corrosponds to self.scan)
        min_idx, max_idx, idx_inc, sect_idx, safe_idx = self.determine_indexing()
        # print(self.ang_inc, self.ang_inc_deg, idx_inc, sect_idx, safe_idx)
        #Initialize max score and group
        max_score = -float("inf")       # Score for closeness correct direction
        max_group = 0                   # Group with best heading
        max_scan = np.array([])
        #Iterate through the sects in the scan
        for group in range(min_idx + safe_idx, max_idx - safe_idx, idx_inc):
            #Loop thorough sect_size scan sections of laserscan
            sect_dists = self.scan[group - sect_idx: group + sect_idx]           #Get sect distances
            sect_safety = self.scan[group - safe_idx: group + safe_idx]     #Get safety region for the sect
            ang_from_goal = abs(self.angs_list[group] - self.goal_ang)                          #Get absolute value of angle from goal
            ang_from_odom = abs(self.angs_list[group])                                          #Get absolute value of angle from current pose angle
            print(len(sect_dists), len(sect_safety))
            #Get the score for this group
            temp_score = self.get_score(sect_dists, sect_safety, ang_from_goal, ang_from_odom)
            print(self.angs_list[group], temp_score, sect_dists.min(), ang_from_goal, ang_from_odom)
            #If this group is better than previous best group, set this group as new max_group
            if temp_score >= max_score:
                max_score = temp_score
                max_group = group
                max_scan = sect_dists

        # Determine heading
        self.determine_heading(max_group)
        self.pub_heading(self.heading)
        # Send steering mesage
        self.get_velocity(max_scan)
        self.check_goal()
        self.create_drive(self.velocity, self.heading)
        self.drive_pub.publish(self.drive_msg)

    def determine_indexing(self):
        min_idx = self.to_usable_angle(self.goal_ang - self.full_view_ang)
        max_idx = self.to_usable_angle(self.goal_ang + self.full_view_ang)
        idx_inc = int(self.sect_spacing/self.ang_inc_deg)       # Spacing angle to indecies
        sect_idx = int(self.sect_size/self.ang_inc_deg)         # Sect angle to indicies
        safe_idx = int(self.safety_size/self.ang_inc_deg)       # Safety angle to indicies
        idx_range = 2*int(self.full_view_ang/self.ang_inc)
        if max_idx - min_idx < idx_range:
            if max_idx == len(self.scan) - 1:
                min_idx = max_idx - idx_range
            if min_idx == 0:
                max_idx = idx_range
        return min_idx, max_idx, idx_inc, sect_idx, safe_idx

    def determine_heading(self, max_group):
        """
        input: the max group from the scoring
        output: none, but sets the heading
        #TODO: Probably should be something with ackermann steering. Currently just points wheels in direction of the max group found.
        """
        self.heading = max(-.34, min(.34, self.angs_list[max_group]))

    def get_clearance(self, scan, dist):
        """
        Input: scan: np array of laser scan distances
               dist: threshold distance
        Output: clearance: proportion (between 0 and 1) of scans in sect that
                           are greater than the threshold distance
        """
        filtered = scan[scan > dist]
        clearance = float(len(filtered))/len(scan)

        return clearance

    def get_score(self, dists, dists_safety, ang_from_goal, ang_from_odom):
        """
        input: dists: numpy array of the distances for the "clearance" section that we will take for scoring
               dists_safety: the dists in the range of values that can cause the sect to be discarded outright
               ang_from_goal: absolute value of angle to the goal
               ang_from_odom: absolute value of angle from current pose angle
        output: The score of a group given the input values

        Score = Kc * clearance - Kg * goal_angle_delta - Kt * turning_angle
        """
        clearance = self.get_clearance(dists, self.clearance_dist)
        safety_clearance = self.get_clearance(dists_safety, self.safety_dist)
        # print "Clearance:", clearance
        # print "Safety clearance:", safety_clearance

        if safety_clearance < self.safety_thresh:
            return -float("inf")
        # Return the weighted score of each relevant input
        return clearance*self.k_dist - ang_from_goal*self.k_goal - ang_from_odom*self.k_goal

    def get_velocity(self, scan):
        """
        Input: np array of a laser scan
        Output: None: sets velocity as a function of how "dense" the local map
                      is. Higher clearance scores -> higher velocities
        """
        # TODO: set the velocity trigger distances as a list in the params
        if self.get_clearance(scan, 2.5) > 0.5:
            self.velocity = self.max_velocity
        elif self.get_clearance(scan, 2) > 0.5:
            self.velocity = self.max_velocity * 0.75
        else:
            self.velocity = self.max_velocity * 0.5

        print "Current velocity:", self.velocity

    def create_drive(self, v, ang):
        """
        create output AckermannDriveStamped mssage
        """
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.header.frame_id = "1"
        self.drive_msg.drive.steering_angle = ang
        self.drive_msg.drive.steering_angle_velocity = 0
        self.drive_msg.drive.speed = 0 if self.in_goal else v
        self.drive_msg.drive.acceleration = 0
        self.drive_msg.drive.jerk = 0

    def to_usable_angle(self, theta):
        '''
        input: theta: angle in racecar's frame
        output: index corresponding to the closest theta from self.ang_list
        '''
        angle_to_min = theta - self.min_ang
        index = int(angle_to_min/self.ang_inc)
        return max(0, min(index, len(self.angs_list) - 2))


    def a_trans(self,data):
        """
        input: data: laserscan message
        output: list of what each angle in the laser scan represents
        """
        #returns [list] of angles within range
        amin = data.angle_min #min angle [rad]
        amax = data.angle_max #max angle [rad]
        ainc = data.angle_increment #min angle increment [rad]
        angs_list = [amin]
        angs_dict = {}
        for i in range(len(data.ranges)):
            angs_list.append(angs_list[i]+ainc)
        return angs_list

    def pub_heading(self, angle):
        """
        Publishes a marker to show the current desired heading of the car.
        Input: angle: relative angle of the next heading in radians
        """
        # arrow marker for current pose
        arrow_msg = Marker()
        arrow_msg.header.frame_id = "/base_link"
        arrow_msg.header.stamp = rospy.Time.now()
        arrow_msg.ns = "arrow_marker"
        arrow_msg.id = 0
        arrow_msg.type = arrow_msg.ARROW

        #start point and end point
        arrow_msg.points = [Point(), Point()]
        #start point
        arrow_msg.points[0].x = 0
        arrow_msg.points[0].y = 0
        arrow_msg.points[0].z = 0
        #end point
        arrow_msg.points[1].x = np.cos(angle)
        arrow_msg.points[1].y = np.sin(angle)
        arrow_msg.points[1].z = 0

        arrow_msg.scale.x = 0.2
        arrow_msg.scale.y = 0.4

        arrow_msg.color.a = 1.0
        arrow_msg.color.g = 1.0

        self.heading_pub.publish(arrow_msg)

        # TODO: implement triangles for visualizing the clearance range and the
        #       safety range
        heading_msg = Marker()
        heading_msg.header.frame_id = "/base_link"
        heading_msg.header.stamp = rospy.Time.now()
        heading_msg.ns = "section_marker"
        heading_msg.id = 0
        heading_msg.type = heading_msg.TRIANGLE_LIST

        # points of chosen slice
        heading_msg.points = [Point(), Point(), Point()]
        # start point on robot
        heading_msg.points[0].x = 0
        heading_msg.points[0].y = 0
        heading_msg.points[0].z = 0
        # laser scan points
        heading_msg.points[1].x = np.cos(angle)
        heading_msg.points[1].y = np.sin(angle)
        heading_msg.points[1].z = 0
        heading_msg.points[2].x = np.cos(angle)
        heading_msg.points[2].y = np.sin(angle)
        heading_msg.points[2].z = 0

        heading_msg.scale.x = 0.2
        heading_msg.scale.y = 0.4

        heading_msg.color.a = 0.5
        heading_msg.color.g = 1.0

        saftey_msg = Marker()



if __name__ == "__main__":
    rospy.init_node('obstacle_avoidance')
    avoidance = ObstacleAvoidance()
    rospy.spin()
