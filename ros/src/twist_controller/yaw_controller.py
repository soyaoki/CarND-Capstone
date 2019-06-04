from math import atan

class YawController(object):
    # Initialization
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle
    
    # Calculate steering value
    def get_steering(self, linear_velocity, angular_velocity, current_velocity):
        # Calculate target yawrate using current velocity
        angular_velocity = current_velocity * angular_velocity / linear_velocity if abs(linear_velocity) > 0. else 0.
        
        # Except very low velocity
        if abs(current_velocity) > 0.1:
            max_yaw_rate = abs(self.max_lat_accel / current_velocity);
            angular_velocity = max(-max_yaw_rate, min(max_yaw_rate, angular_velocity))
        # Return steering angle to get the target yawrate
        return self.get_angle(max(current_velocity, self.min_speed) / angular_velocity) if abs(angular_velocity) > 0. else 0.0;
    
    def get_angle(self, radius): # radius = radius of curvature
        angle = atan(self.wheel_base / radius) * self.steer_ratio
        return max(self.min_angle, min(self.max_angle, angle))
