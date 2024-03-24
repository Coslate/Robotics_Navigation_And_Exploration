import sys
import numpy as np
import math
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerPurePursuitBicycle(Controller):
    def __init__(self, kp=1, Lfc=25):
        self.path = None
        self.kp = kp
        self.Lfc = Lfc

    def normalize_angle(self, angle):
        #normalize the angle to [-np.pi, np.pi]
        '''
        a_mod = math.fmod(angle+180, 2*180)
        if a_mod < 0.0:
            a_mod += 2.0*180

        return a_mod - 180
        '''

        return (angle + 180) % 360 - 180

    # State: [x, y, yaw, v, l]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None

        # Extract State
        x, y, yaw, v, l = info["x"], info["y"], info["yaw"], info["v"], info["l"]

        # Search Front Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        Ld = self.kp*v + self.Lfc
        target_idx = min_idx
        for i in range(min_idx,len(self.path)-1):
            dist = np.sqrt((self.path[i+1,0]-x)**2 + (self.path[i+1,1]-y)**2)
            if dist > Ld:
                target_idx = i
                break
        target = self.path[target_idx]

        # TODO: Pure Pursuit Control for Bicycle Kinematic Model
        xg  = target[0]
        yg  = target[1]
        yaw_norm = self.normalize_angle(yaw)
        ang = self.normalize_angle(np.rad2deg(np.arctan2(yg-y, xg-x)) - yaw_norm)
        next_delta = np.rad2deg(np.arctan2(2.0*l*np.sin(np.deg2rad(ang)), Ld))

        return next_delta, target
