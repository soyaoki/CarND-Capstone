#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
from PIL import Image as Img
from timeit import default_timer as timer
import random

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    # Initialization
    def __init__(self):
        rospy.init_node('tl_detector')
        # Variables
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.lights = []
        self.waypoints_2d = None
        self.waypoint_tree = None
        
        # Subscriber
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # Get current pose -> self.pose
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # Get waypoints -> self.waypoints

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb) # Get light state and current pose from vehicle msg.lights -> self.light (IT CAN BE USED IN SIMULATOR ONLY !)
        # "TrafficLightsArray" containts:
        # TrafficLight[] lights
            # Header header
            # geometry_msgs/PoseStamped pose
            # uint8 state
            
            # uint8 UNKNOWN=4
            # uint8 GREEN=2
            # uint8 YELLOW=1
            # uint8 RED=0
            
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb) # Get front image -> self.camera_image
        # "Image" containts:
        # 
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        # Pubilsher
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        # Variables
        self.bridge = CvBridge() # Convert ROS_image -> Opencv_image
        self.light_classifier = TLClassifier() # Classify traffic light state
        self.listener = tf.TransformListener() # transform Coordinate

        self.state = TrafficLight.UNKNOWN # 4
        self.last_state = TrafficLight.UNKNOWN # 4
        self.last_wp = -1 # last waypoint
        self.state_count = 0 # Count that image was classified as same traffic continuously
        
        # Loop
        self.loop()
    
    # Call back functions
    def pose_cb(self, msg):
        self.pose = msg
        #rospy.logwarn("Tl detector: subscribed 'current pose'.")

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        #rospy.logwarn("Tl detector: subscribed 'base waypoints'.")

    def traffic_cb(self, msg):
        self.lights = msg.lights
        #rospy.logwarn("Tl detector: subscribed 'traffic lights'.")

    # Get and classify image
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        #rospy.logwarn("Tl detector: subscribed 'image color'.")
        
    # LOOP
    def loop(self):
        # Excute rate
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            start = timer()
            if (self.pose):
                light_wp, state = self.process_traffic_lights() # Detect and classify trafic light. Return light waypoint and state
                # Camera On - > Use light state classied by image
                # Camera Off -> Use light state information from simulator                
            
                '''
                Publish upcoming red lights at camera frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times till we start using it. Otherwise the previous stable state is
                used.
                '''
                # If state change
                if self.state != state:
                    self.state_count = 0 # Reset count
                    self.state = state # Update state
                # If result of traffic light classification is same over threshold continuously
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state # Update last state
                    light_wp = light_wp if state == TrafficLight.RED else -1 # If traffic light state is red, retun light waypoint to stop vehicle 
                    self.last_wp = light_wp # Update light waypoint
                    self.upcoming_red_light_pub.publish(Int32(light_wp)) # Publish light waypoint
                else:
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp)) # Publish last waypoint (-1 or red light waypoint)
                self.state_count += 1
                #rospy.logwarn("Tl detector: published 'traffic waypoint'(stop line index).")
                end = timer()
                #rospy.logwarn("Traffic light recognition time : {0}".format(end-start) + ", Closest light state : {0}".format(state))
            rate.sleep()
    
    # Detect and classify trafic light. Return light waypoint and state
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Variables to return
        light = None
        line_wp_idx = -1
        
        # Get closest waypint index
        car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        # Use light information in Simulator
        if(self.lights):
            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            # For each lights
            for i, light in enumerate(self.lights):
                # Get stop line waypoints index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1]) # closest waypoint indx for the stop line
                # Find index of closest stop line waypoint
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx
            # If closest light exist, return the light's state and waypoint
            if closest_light:
                # Randomly use image
                if (random.randint(0,100)==10):
                    if (self.has_image):
                        state = self.get_light_state()
                        return line_wp_idx, state
                return line_wp_idx, closest_light.state
            
        # When system can not use light information, Classify light state using camera image
        elif (self.has_image):
            state = self.get_light_state()
            if(state == TrafficLight.RED):
                line_wp_idx = car_wp_idx + 5 # Turning is needed
            return line_wp_idx, state
        
        return -1, TrafficLight.UNKNOWN

    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return closest_idx

    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """        
        # Convert ROS image to CV image
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        
        # Convert CV image to PIL
        PIL_image = Img.fromarray(cv_image)
        
        #Get classification the light state
        return self.light_classifier.get_classification(PIL_image)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
