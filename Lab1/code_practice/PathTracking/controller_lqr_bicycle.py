import sys
import numpy as np
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBicycle(Controller):
    def __init__(self, Q=np.eye(4), R=np.eye(1)):
        self.path = None
        self.Q = Q
        self.Q[0,0] = 1000
        self.Q[1,1] = 10
        self.Q[2,2] = 10
        self.Q[3,3] = 10
        self.R = R*5000
        self.pe = 0
        self.pth_e = 0
        self.theta_p_last = 0

    def set_path(self, path):
        super().set_path(path)
        self.pe = 0
        self.pth_e = 0

    def _solve_DARE(self, A, B, Q, R, max_iter=150, eps=0.01): # Discrete-time Algebra Riccati Equation (DARE)
        P = Q.copy()
        for i in range(max_iter):
            temp = np.linalg.inv(R + B.T @ P @ B)
            Pn = A.T @ P @ A - A.T @ P @ B @ temp @ B.T @ P @ A + Q
            if np.abs(Pn - P).max() < eps:
                break
            P = Pn
        return Pn

    def normalize_angle(self, angle):
        #normalize the angle to [-np.pi, np.pi]
        '''
        a_mod = math.fmod(angle+180, 2*180)
        if a_mod < 0.0:
            a_mod += 2.0*180

        return a_mod - 180
        '''

        return (angle + 180) % 360 - 180

    # State: [x, y, yaw, delta, v, l, dt]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None

        # Extract State
        x, y, yaw, delta, v, l, dt, w = info["x"], info["y"], info["yaw"], info["delta"], info["v"], info["l"], info["dt"], info["w"]
        yaw = utils.angle_norm(yaw)
        front_x = x + l*np.cos(np.deg2rad(yaw))
        front_y = y + l*np.sin(np.deg2rad(yaw))
        vf = v / np.cos(np.deg2rad(delta))

        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])

        # TODO: LQR Control for Bicycle Kinematic Model
        xg      = target[0]
        yg      = target[1]
        theta_p = target[2]
        theta_p_dot = utils.angle_norm((theta_p - self.theta_p_last))/dt
        theta_e = utils.angle_norm(theta_p-yaw)
        theta_p_plus90 = utils.angle_norm(theta_p+90)

        ang = utils.angle_norm(np.rad2deg(np.arctan2(yg-y, xg-x)) - yaw)
        e = min_dist * np.sin(np.deg2rad(ang))
        edot = -vf * np.sin(np.deg2rad(utils.angle_norm(delta - theta_e)))
        theta_e_dot = theta_p_dot - w

        xt = np.array([[e      ],
                       [edot   ],
                       [theta_e],
                       [theta_e_dot]])

        A = np.array([[1, dt, 0,  0],
                      [0,  0, vf, 0],
                      [0,  0, 1, dt],
                      [0,  0, 0,  0]])

        B = np.array([[0],
                      [0],
                      [0],
                      [-v/l]])

        P_opt = self._solve_DARE(A, B, self.Q, self.R)
        tmp_inv = np.linalg.inv(self.R + B.T @ P_opt @ B)
        u_opt = -1*tmp_inv @ B.T @ P_opt @ A @ xt

        next_delta = u_opt[0][0]
        self.theta_p_last = theta_p

        print(f"")
        print(f"e           = {e}")
        print(f"edot        = {edot}")
        print(f"theta_e     = {theta_e}")
        print(f"theta_e_dot = {theta_e_dot}")
        print(f"next_delta  = {next_delta}")
        return next_delta, target
