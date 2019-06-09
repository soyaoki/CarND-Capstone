#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    # Initialization
    def __init__(self):
        rospy.init_node('dbw_node')
        # Get parameters
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        
        # Publisher
        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
        # "SteeringCmd" containts:
        #     float32 steering_wheelangle_cmd
        #     float32 steering_wheelangle_velocity
        #     bool enable
        #     bool clear
        #     bool ignore
        #     uint8 count
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd, queue_size=1)
        # "ThrottleCmd" containts:
        #     uint8 CMD_NODE
        #     uint8 CMD_PEDAL
        #     uint8 CMD_PERCENT
        #     float32 pedal_cmd
        #     uint8 pedal_cmd_type
        #     bool enable
        #     bool clear
        #     bool ignore
        #     uint8 count
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd, queue_size=1)
        # "BrakeCmd" containts:
        #     uint8 CMD_NODE
        #     uint8 CMD_PEDAL
        #     uint8 CMD_PERCENT
        #     uint8 CMD_TORQUE
        #     float32 TORQUE_BOO
        #     float32 TORQUE_MAX
        #     float32 pedal_cmd
        #     uint8 pedal_cmd_type
        #     bool boo_cmd
        #     bool enable
        #     bool clear
        #     bool ignore
        #     uint8 count

        # TODO: Create `Controller` object from twist_controller
        self.controller = Controller(vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     brake_deadband=brake_deadband,
                                     decel_limit=decel_limit,
                                     accel_limit=accel_limit,
                                     wheel_radius=wheel_radius,
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle)

        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb) # IMPORTANT
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb) # Get target velocity and angular from pure persuit
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb) # Get current velocity
        # "TwistStamped" containts:
        # twist(TwistStamped) ------ header(seq, stamp, frame_id)
        #                              |------ twist(Twist) ------ linear(x,y,z)
        #                                                       | ----- angular(x,y,z)
        
        self.dbw_enabled = None
        self.linear_vel = None
        self.angular_vel = None
        self.current_vel = None
        self.throttle = self.steering = self.brake = 0
        
        # LOOP
        self.loop()
    
    # LOOP
    def loop(self):
        rate = rospy.Rate(50) # Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # If current velocity, target velocity and angular exist, controller calculate control values
            if not None in (self.current_vel, self.linear_vel, self.angular_vel):
                # Calculate throttle, brake and steering value for control
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel, self.dbw_enabled, self.linear_vel, self.angular_vel)
                # self.steering = 1.0 # test
                # If Drive by wire is enable, publish control values
            if self.dbw_enabled:
                self.publish(self.throttle, self.brake, self.steering)
                #rospy.logwarn("Dbw node: published 'brake cmd', 'steering cmd' and 'throttle cmd'.")
            rate.sleep()
        
    # Call back functions
    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg
        #rospy.logwarn("Dbw node: subscribed 'dbw enabled'.")

    def twist_cb(self, msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z
        #rospy.logwarn("Dbw node: subscribed 'twist cmd'.")

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x
        #rospy.logwarn("Dbw node: subscribed 'current velocity'.")
        
    # Publish control value
    def publish(self, throttle, brake, steer):
        # Create throttle command
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)
        
        # Create steering command
        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)
        
        # Create brake command
        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

if __name__ == '__main__':
    DBWNode()
