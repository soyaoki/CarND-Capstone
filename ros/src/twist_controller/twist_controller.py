from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    # Initialization
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        # Create controller for steering 
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        # Create controller for throttle 
        kp = 0.3
        ki = 0.1
        kd = 0.
        mn = 0. # Min acc
        mx = 0.2 # MAX acc
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        
        # Create low-pass filter for velocity
        tau = 0.5
        ts = 0.02
        self.vel_lpf = LowPassFilter(tau, ts)
        
        # Variables
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle

        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        #rospy.logwarn("DBW_enabled: {0}".format(dbw_enabled))
        
        # If DBE_enable is None, system doesnt control
        if not dbw_enabled:
            self.throttle_controller.reset() # I_error in PID reset
            return 0.0, 0.0, 0.0
        # Decrease velocity noise
        current_vel = self.vel_lpf.filt(current_vel)
        
        #rospy.logwarn("Angular vel: {0}".format(angular_vel))
        #rospy.logwarn("Target vel: {0}".format(linear_vel))
        #rospy.logwarn("Target angular vel: {0}".format(angular_vel))
        #rospy.logwarn("Current vel: {0}".format(current_vel))
        #rospy.logwarn("Filtered vel: {0}".format(self.vel_lpf.get()))
        
        # Calculate steering value
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        #rospy.logwarn("Steering angle: {0}".format(steering))

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel
        
        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 400
        elif throttle < 0.1 and vel_error < 0:
            throttle= 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius # Trque N*m
        
        #rospy.logwarn("Throttle: {0}".format(throttle))
        #rospy.logwarn("Brake: {0}".format(brake))
        return throttle, brake, steering
