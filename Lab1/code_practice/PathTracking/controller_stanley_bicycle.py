import sys
import numpy as np
import math
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerStanleyBicycle(Controller):
    def __init__(self, kp=0.5):
        self.path = None
        self.kp = kp

    def normalize_angle(self, angle):
        #normalize the angle to [-np.pi, np.pi]
        '''
        a_mod = math.fmod(angle+180, 2*180)
        if a_mod < 0.0:
            a_mod += 2.0*180

        return a_mod - 180
        '''

        return (angle + 180) % 360 - 180

    # State: [x, y, yaw, delta, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None

        # Extract State
        x, y, yaw, delta, v, l = info["x"], info["y"], info["yaw"], info["delta"], info["v"], info["l"]

        # Search Front Wheel Target
        front_x = x + l*np.cos(np.deg2rad(yaw))
        front_y = y + l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta))
        min_idx, min_dist = utils.search_nearest(self.path, (front_x,front_y))
        target = self.path[min_idx]

        # TODO: Stanley Control for Bicycle Kinematic Model
        xg  = target[0]
        yg  = target[1]
        yaw_norm = self.normalize_angle(yaw)
        theta_p = target[2]
        theta_e = self.normalize_angle(theta_p-yaw_norm)
        theta_p_plus90 = self.normalize_angle(theta_p+90)
        e = np.dot(np.array([(front_x-xg), (front_y-yg)]), np.array([np.cos(np.deg2rad(theta_p_plus90)), np.sin(np.deg2rad(theta_p_plus90))]))
        next_delta = self.normalize_angle(np.rad2deg(np.arctan2(-1*self.kp*e, vf)) + theta_e)
        return next_delta, target
