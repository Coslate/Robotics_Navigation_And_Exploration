import sys
import numpy as np
sys.path.append("..")
import PathTracking.utils as utils
from PathTracking.controller import Controller

class ControllerLQRBasic(Controller):
    def __init__(self, Q=np.eye(4), R=np.eye(1)):
        self.path = None
        self.Q = Q
        self.Q[0,0] = 1
        self.Q[1,1] = 1
        self.Q[2,2] = 1
        self.Q[3,3] = 1
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

    # State: [x, y, yaw, delta, v, l, dt]
    def feedback(self, info):
        # Check Path
        if self.path is None:
            print("No path !!")
            return None, None

        # Extract State
        x, y, yaw, v, dt, w = info["x"], info["y"], info["yaw"], info["v"], info["dt"], info["w"]
        yaw = utils.angle_norm(yaw)

        # Search Nesrest Target
        min_idx, min_dist = utils.search_nearest(self.path, (x,y))
        target = self.path[min_idx]
        target[2] = utils.angle_norm(target[2])

        # TODO: LQR Control for Basic Kinematic Model
        xg      = target[0]
        yg      = target[1]
        theta_p = target[2]
        ang = utils.angle_norm(np.rad2deg(np.arctan2(yg-y, xg-x)) - yaw)
        e   = min_dist * np.sin(np.deg2rad(ang))
        edot= v * np.sin(np.deg2rad(yaw)) * np.tan(np.deg2rad(ang))
        theta_p_dot = utils.angle_norm((theta_p - self.theta_p_last))/dt
        theta_e = utils.angle_norm(theta_p-yaw)
        theta_e_dot = theta_p_dot - w
        eps = 1e-20
        xt = np.array([[e      ],
                       [edot   ],
                       [theta_e],
                       [theta_e_dot]])

        A = np.array([[1, dt, 0, 0],
                      [0,  0, v, 0],
                      [0,  0, 1, dt],
                      [0,  0, 0, theta_p_dot/(eps+theta_e_dot)]])

        B = np.array([[0],
                      [0],
                      [0],
                      [-1]])

        P_opt = self._solve_DARE(A, B, self.Q, self.R)
        temp = np.linalg.inv(self.R + B.T @ P_opt @ B)
        u_opt = -1*temp @ B.T @ P_opt @ A @ xt

        next_w = u_opt[0][0]
        self.theta_p_last = theta_p
        return next_w, target
