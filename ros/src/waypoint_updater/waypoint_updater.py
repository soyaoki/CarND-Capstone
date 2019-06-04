#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5*9.8 # Max deceleration [m/s2]

class WaypointUpdater(object):
    # Initialization
    def __init__(self):
        # Node
        rospy.init_node('waypoint_updater')
        
        # TODO: Add other member variables you need below
        self.pose = None 
        self.base_lane = None 
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_idx = -1 
        #self.obstacle_wp_idx = -1
        
        # Subscriber
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # Get Pose msg -> self.pose
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # Get lane(waypoints) msg -> self.base_waypoints
        # Msg "Lane" containts:
        # header(seq, stamp, frame_id)
        # waypints
        #      |------ pose(PoseStamped) ------ header(seq, stamp, frame_id)
        #      |                                   |------ pose(Pose) ------ position(x,y,z)
        #      |                                                           | ----- orientation(x,y,z,w)
        #      |------ twist(TwistStamped) ------ header(seq, stamp, frame_id)
        #                                          |------ twist(Twist) ------ linear(x,y,z)
        #                                                                   | ----- angular(x,y,z)
        
        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) # Stopline Index in waypints
        # rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb) # Obstacle Index in waypints
        
        # LOOP
        self.loop()
    
    # Call back functions
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        #rospy.logwarn("Waypoint updater: subscribed 'current pose'.")

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        #rospy.logwarn("Waypoint updater: subscribed 'base waypoints'.")
            
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        #rospy.logwarn("Waypoint updater: subscribed 'traffic waypoint'.")

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        self.obstacle_wp_idx = msg.data
        #rospy.logwarn("Waypoint updater: subscribed 'obstacle waypoint'.")

    # LOOP
    def loop(self):
        # Excute rate
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # If pose and base_waypoints exist, publish waypoints
            if self.pose and self.base_lane:
                self.publish_waypoints()
                #rospy.logwarn("Waypoint updater: published 'final waypoints'.")
            rate.sleep()
        
    # Publish waypoints
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
    
    # Generate lane
    def generate_lane(self):
        lane = Lane() #waypoints(pose, twist)
        
        # Get the closest waypoint index
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS # +200
        # Cut out waypoints in needed area
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
        # If stop line not exist (idx == -1) or stop line exist farther than farthest index, give base_waypoints to lane
        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        # Else stop line exist in base_waypoints, create waypoints to decelerate and stop before stop line
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx) # deceleration
        return lane
    
    # Get the closest waypoint index
    def get_closest_waypoint_idx(self):
        # Get current position
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # Get closest point from current position
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        
        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        # Calculate inner product
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        # If inmer product is positive, closest_idx exist behind the vehicle
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        #rospy.logwarn("X: {0}".format(x))
        #rospy.logwarn("Y: {0}".format(y))
        #rospy.logwarn("Closest_idx: {0}".format(closest_idx))
        #rospy.logwarn("Closest_X: {0}".format(cl_vect[0]))
        #rospy.logwarn("Closest_Y: {0}".format(cl_vect[1]))
            
        return closest_idx
    
    # Create waypoints to decelerate and stop before stop line
    def decelerate_waypoints(self, waypoints, closest_idx):
        # Create new waypoints
        temp = []
        # Each waypoint(Pose and Twist)
        for i, wp in enumerate(waypoints):
            # Create a new waypoint
            p = Waypoint()
            p.pose = wp.pose # Copy pose(PoseStamped)
            # Define stop line index
            stop_idx = max(self.stopline_wp_idx - closest_idx - 5, 0)
            # Calculate distance 
            dist = self.distance(waypoints, i, stop_idx)
            # Calculate velocity for deceleration
            vel_2order_dec = math.sqrt(2 * MAX_DECEL * dist)
            if vel_2order_dec < 1.0:
                vel_2order_dec = 0.0
            # If velocity for deceleration is faster than current velocity, don't change anyting
            p.twist.twist.linear.x = min(vel_2order_dec, wp.twist.twist.linear.x)
            # Add velocity profile to new waypoints
            temp.append(p)
        return temp

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            #rospy.logwarn("Waypoint 1: {0}".format(wp1))
            #rospy.logwarn("Waypoint 2: {0}".format(wp2))
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
