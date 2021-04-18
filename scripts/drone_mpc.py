from cvxpy import *
import numpy as np
import scipy.signal
import scipy.optimize
from scipy import sparse
from scipy.spatial.transform import Rotation
import math


class DroneMPC(object):
    def __init__(self, N):
        # Continuous Flat Dynamics for quadrotor
        # states: x y z vx vy vz yaw
        # controls: ax ay az yaw_rate
        self.N = N
        self.dt = 0.1
        self.t_predict = np.arange(0, N * self.dt, self.dt)
        self.FLAT_STATES = 7
        self.FLAT_CTRLS = 4
        self.A = np.eye(self.FLAT_STATES)
        self.A[0:3, 3:6] = np.eye(3) * self.dt
        print(self.A)
        self.B = np.zeros((self.FLAT_STATES, self.FLAT_CTRLS), dtype=np.float16)
        self.B[:3, :3] = 0.5 * np.eye(3) * self.dt ** 2
        self.B[3:, :] = np.eye(4) * self.dt
        print(self.B)
        self.A_bar = self._build_a_bar(self.A)
        self.B_bar = self._build_b_bar(self.A, self.B)
        # unused, necessary input into scipy state dynamics
        # self.C = np.zeros((1, self.FLAT_STATES), dtype=np.float16)
        # self.D = np.zeros((1, self.FLAT_CTRLS), dtype=np.float16)
        # self.sys = scipy.signal.StateSpace(self.A, self.B, self.C, self.D)
        self.Gff = np.array([[0, 0, 9.81, 0]]).T  # gravity compensation

        # Constraints
        self.umin = np.array([-2, -2, -2, -np.Inf])
        self.umax = np.array([2, 2, 2, np.Inf])
        self.xmin = np.array([-10, -10, 6, -2, -2, -2, -np.Inf])
        self.xmax = np.array([10, 10, 10, 2, 2, 2, np.Inf])
        self.state_constraints = scipy.optimize.LinearConstraint(
            A=np.eye(self.FLAT_STATES), lb=self.xmin, ub=self.xmax, keep_feasible=False)
        # self.constraints = [self.state_constraints,]

        # Quadratic costs
        self.Q = np.diag([20, 20, 20, 0.1, 0.1, 0.1, 1])
        self.Q_dist = np.diag([1., 1., 1.])
        self.R = np.diag([1, 1, 1, 0.001])
        self.S = self.Q * 10

        # Lifted Quadratic costs over horizon N
        self.Q_bar = scipy.linalg.block_diag(*tuple([self.Q] * (N + 1)))
        self.Q_bar[-self.FLAT_STATES - 1:-1, -self.FLAT_STATES - 1:-1] = self.S
        self.R_bar = scipy.linalg.block_diag(*tuple([self.R] * N))

    @staticmethod
    def extend(u, k):
        """We optimize over original horizon N, but may want to visualize
        path over additional horizon N + k

        Returns (N+k x self.FLAT_CTRLS)
        """
        u_repeat = np.repeat(u[-1, :].reshape(1, 1), k, axis=0)
        return np.vstack([u, u_repeat])

    def prediction(self, U, t, x0):
        """Predict the effect of an control series U"""
        # t, y, x = scipy.signal.lsim(self.sys, U, t, X0=x0, interp=False)
        xs = self.A_bar @ x0 + self.B_bar @ U
        return xs.reshape(self.N + 1, self.FLAT_STATES)

    def state_cost(self, xt, xt_target, target_pixel, phi):
        """
        target_pixel is the 3D coordinate of a pixel in the drone's frame. Needs to be
        transformed into world frame
        """
        yaw = xt[-1]
        # TODO: Check if np and Rotation can handle these variables
        # vel = np.array([xt[3], xt[4], xt[5]])
        # vel = vel / np.linalg.norm(vel)
        # # TODO: check if vel[1] or [2]!!
        # theta = math.asin(-vel[2])  # pitch
        # phi = 0  # cannot be determined from velocity
        # (R @ X + T) - T
        vec_target = Rotation.from_euler('ZYX', [yaw, 0, 0]).as_matrix() @ target_pixel
        vec_target /= np.linalg.norm(vec_target)
        vec_cur = xt_target - xt[:3]
        vec_cur /= np.linalg.norm(vec_cur)

        # want vectors to align (dot-prod = +1)

        desired_height_cost = (xt[0] - 8) ** 2
        viewpoint_cost = (1 - vec_target @ vec_cur) ** 2
        horiz_dist = np.linalg.norm(xt[:2] - xt_target[:2]) ** 2
        desired_horiz_dist = 22
        desired_dist_cost = (desired_horiz_dist - horiz_dist) ** 2

        return 1000 * viewpoint_cost + desired_height_cost + desired_dist_cost

    @staticmethod
    def objective(U, self, t, x0, target_traj, target_pixel, phi0):
        X = self.prediction(U, t, x0)
        control_cost = U.T @ self.R_bar @ U
        # TODO: Double-check indexing! Does X[i] match with target_traj[i]?
        viewpoint_cost = sum([self.state_cost(X[i], target_traj[i], target_pixel, phi0)
                              for i in range(self.N)])
        return 0.01 * control_cost + 10 * viewpoint_cost
        # constraintpenalty = sum(umag[umag > 2])
        # movepenalty = sum(np.abs(np.diff(u)))
        # strongfinish = np.abs(y[-1] - r[-1])
        #     import ipdb
        #     ipdb.set_trace()
        # return sum((r - y) ** 2) + 0.1 * constraintpenalty + 0.1 * movepenalty + 0 * strongfinish

    def solve(self, x0, u0, target_pixel, target_traj, user_sq_dist, phi0):
        U = np.ones((self.N * self.FLAT_CTRLS, 1))  # flattened out controls
        args = (self, self.t_predict, x0, target_traj, target_pixel, phi0)
        result = scipy.optimize.minimize(DroneMPC.objective, U, args=args)
        Uopt = result.x
        return self.prediction(Uopt, self.t_predict, x0), Uopt.reshape(self.N, self.FLAT_CTRLS)
        # result.fun

    @staticmethod
    def inverse_dyn(q, x_ref, u):
        up = u[:3]  # linear accelerations
        # up = np.array(
        #     np.ndarray.flatten(up).tolist()[0])

        # thrust = float(m * np.linalg.norm(up))
        normal_measured = Rotation.from_quat(q).apply([0, 0, 1])
        thrust = max(0, np.dot(up, normal_measured))
        psid = x_ref[6]
        rotz = Rotation.from_euler(
            seq="ZYX", angles=[-psid, 0, 0]).as_dcm()
        z = rotz.dot(up) / np.linalg.norm(up)
        phid = -math.atan2(z[1], z[2])
        thetad = math.atan2(z[0], z[2])
        return [thrust, phid, thetad, psid]

    def _build_a_bar(self, A):
        rm = A.shape[0]
        cm = A.shape[1]
        A_bar = np.zeros((rm * (self.N + 1), cm))
        for i in range(self.N + 1):
            A_bar[rm * i:rm * (i + 1), :] = np.linalg.matrix_power(A, i)
        return A_bar

    def _build_b_bar(self, A, B):
        rm = B.shape[0]
        cm = B.shape[1]
        B_bar = np.zeros((rm * (self.N + 1), cm * self.N))
        for r in range(self.N + 1):
            for c in range(self.N):
                order = r - c - 1
                if order < 0:
                    B_bar[rm * r:rm * (r + 1), cm * c:cm * (c + 1)] = np.zeros(B.shape)
                else:
                    B_bar[rm * r:rm * (r + 1), cm * c:cm * (c + 1)] = np.dot(np.linalg.matrix_power(A, order), B)
        return B_bar
