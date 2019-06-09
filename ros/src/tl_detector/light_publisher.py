#!/usr/bin/env python
import rospy
import tf
import cv2
import time
from styx_msgs.msg import TrafficLightArray, TrafficLight
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped

import numpy as np
import rospkg
import math

class TLPublisher(object):
    def __init__(self):
        rospy.init_node('tl_publisher')
        # Publisher
        self.traffic_light_pubs = rospy.Publisher('/vehicle/traffic_lights', TrafficLightArray, queue_size=1)
        # Initialize
        lights = TrafficLightArray() # Create "Traffic Light Array" msg
        light = self.create_light(20.991, 22.837, 1.524, 0.08, 3) # Create "Traffic Light" msg
        lights.header = light.header
        lights.lights = [light]
        self.lights = lights
        # LOOP
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.traffic_light_pubs.publish(self.lights)
            rate.sleep()

    def create_light(self, x, y, z, yaw, state):
        light = TrafficLight()

        light.header = Header()
        light.header.stamp = rospy.Time.now()
        light.header.frame_id = '/world'

        light.pose = self.create_pose(x, y, z, yaw)
        light.state = state

        return light

    def create_pose(self, x, y, z, yaw=0.):
        pose = PoseStamped()

        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/world'

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = tf.transformations.quaternion_from_euler(0., 0., math.pi * yaw/180.)
        pose.pose.orientation = Quaternion(*q)

        return pose

if __name__ == '__main__':
    try:
        TLPublisher()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic publisher node.')
