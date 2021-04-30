import numpy as np
from scipy.spatial.transform import Rotation
import math

import casadi
import forcespro.dump
import forcespro.nlp
import sys


class DroneMPC(object):
    desired_horiz_dist = 10  # m
    desired_height = 8  # m

    # Quadratic costs
    Q = np.diag([20, 20, 20, 0.1, 0.1, 0.1, 1])
    R = np.diag([1, 1, 1, 0.001])

    def __init__(self, N=10, dt=0.1, load_solver=True, solver_dir="drone_mpc_compiled"):
        # Continuous Flat Dynamics for quadrotor
        # states: x y z vx vy vz yaw
        # controls: ax ay az yaw_rate
        self.N = N
        self.dt = dt
        self.t_predict = np.arange(0, N * self.dt, self.dt)
        self.state_dim = 7
        self.ctrl_dim = 4
        self.A = np.eye(self.state_dim)
        self.A[0:3, 3:6] = np.eye(3) * self.dt
        self.B = np.zeros((self.state_dim, self.ctrl_dim))
        self.B[:3, :3] = 0.5 * np.eye(3) * self.dt ** 2
        self.B[3:, :] = np.eye(4) * self.dt
        self.Gff = np.array([[0, 0, 9.81, 0]]).T  # gravity compensation

        # Constraints
        # Upper/lower variable bounds lb <= eval_state_constraints(z) <= ub
        # https://forces.embotech.com/Documentation/high_level_interface/index.html?highlight=symbolicmodel#inequalities
        max_yaw_rate = (2 * math.pi / 3) * self.dt  # max rotate rad/sec * (dt sec / 1 step)
        max_horiz_acc = 1 * self.dt  # m/s^2 * (dt sec / 1 step)
        max_vert_acc = 2 * self.dt  # m/s^2 * (dt sec / 1 step)
        # ax  ay  az  yaw_rate
        self.ctrl_lb = np.array([-max_horiz_acc, -max_horiz_acc, -max_vert_acc, -max_yaw_rate])
        self.ctrl_ub = np.array([max_horiz_acc, max_horiz_acc, max_vert_acc, max_yaw_rate])

        max_horiz_vel = 5  # m/s
        max_vert_vel = 5  # m/s
        min_yaw = -np.Inf
        max_yaw = np.Inf
        min_y = 0  # TODO!!!
        max_y = 6
        min_x = -6
        max_x = 6
        min_z = 6
        max_z = 10
        # x  y  z  vx  vy  vz  yaw
        self.state_lb = np.array([min_x, min_y, min_z, -max_horiz_vel, -max_horiz_vel, -max_vert_vel, min_yaw])
        self.state_ub = np.array([max_x, max_y, max_z, max_horiz_vel, max_horiz_vel, max_vert_vel, max_yaw])

        # actually weigh the terminal cost at time N lower since target pose estimate is less reliable
        # self.terminal_cost = 0.5

        # Forces Pro solver
        self.model = None
        self.generate_model()

        self.solver = None
        if load_solver:
            self.solver = forcespro.nlp.Solver.from_directory(solver_dir)
        else:
            self.generate_solver(solver_dir)

    @staticmethod
    def inverse_dyn(q, x_ref, u):
        up = u[:3]  # linear accelerations
        normal_measured = Rotation.from_quat(q).apply([0, 0, 1])
        thrust = max(0, np.dot(up, normal_measured))
        psid = x_ref[6]
        rotz = Rotation.from_euler(
            seq="ZYX", angles=[-psid, 0, 0]).as_matrix()
        z = rotz.dot(up) / np.linalg.norm(up)
        phid = -math.atan2(z[1], z[2])
        thetad = math.atan2(z[0], z[2])
        return [thrust, phid, thetad, psid]

    @staticmethod
    def norm(vec):
        return (vec[0] ** 2 + vec[1] ** 2 + vec[2] ** 2) ** 0.5

    @staticmethod
    def state_cost(z, args):
        """
        target_pixel is the 3D coordinate of a pixel in the drone's frame. Needs to be
        transformed into world frame
        """
        xt_target, target_pixel = args[:3], args[3:]
        xt = z[4:]
        u = z[:4]
        yaw = xt[-1]
        rot = np.array([
            [1, 0, 0.0],
            [0, casadi.cos(yaw), -casadi.sin(yaw)],
            [0, casadi.sin(yaw), casadi.cos(yaw)]
        ])
        # desired 3D vector through center pixel of image
        vec_target = casadi.mtimes(rot, target_pixel)
        vec_target = vec_target / DroneMPC.norm(vec_target)

        # actual 3D vector from drone camera center to target
        vec_cur = xt_target - xt[:3]
        vec_cur = vec_cur / DroneMPC.norm(vec_cur)

        # want vectors to align (dot-prod = +1)
        viewpoint_cost = (1 - casadi.mtimes(vec_target.T, vec_cur)) ** 2

        # maintain desired height
        desired_height_cost = (xt[0] - DroneMPC.desired_height) ** 2

        # maintain desired horiz distance from target
        horiz_dist = (xt[0] - xt_target[0]) ** 2 + (xt[1] - xt_target[1]) ** 2
        desired_dist_cost = (DroneMPC.desired_horiz_dist - horiz_dist) ** 2

        control_cost = casadi.mtimes([u.T, DroneMPC.R, u])

        return 100 * viewpoint_cost + 10 * desired_height_cost + 10 * desired_dist_cost + 0.01 * control_cost

    def generate_model(self):
        self.model = forcespro.nlp.SymbolicModel()  # create empty model
        self.model.N = self.N  # horizon length
        self.model.nvar = self.ctrl_dim + 7  # number of variables
        self.model.neq = 7  # number of equality constraints
        self.model.npar = 6  # number of runtime parameters
        self.model.objective = self.state_cost
        self.model.objectiveN = self.state_cost
        self.model.eq = lambda z: self.A @ z[4:] + self.B @ z[:4]
        self.model.E = np.concatenate([np.zeros((7, 4)), np.eye(7)], axis=1)

        # State and contrl constraints
        # self.model.hl = np.concatenate([ctrl_lb, state_lb])
        # self.model.hu = np.concatenate([ctrl_ub, state_ub])
        # self.model.ineq = eval_state_constraints
        self.model.lb = np.concatenate([self.ctrl_lb, self.state_lb])
        self.model.ub = np.concatenate([self.ctrl_ub, self.state_ub])

        self.model.xinitidx = range(4, 11)

    def generate_solver(self, solver_dir):
        codeoptions = forcespro.CodeOptions(solver_dir)
        codeoptions.maxit = 1000  # Maximum number of iterations
        codeoptions.printlevel = 0  # Use printlevel = 2 to print progress (but
        #                             not for timings)
        codeoptions.optlevel = 0  # 0 no optimization, 1 optimize for size,
        #                             2 optimize for speed, 3 optimize for size & speed
        codeoptions.cleanup = False
        codeoptions.timing = 1
        # codeoptions.nlp.hessian_approximation = 'bfgs'
        # codeoptions.solvemethod = 'PDIP_NLP' # choose the solver method Sequential
        #                              Quadratic Programming
        # codeoptions.sqp_nlp.maxqps = 1      # maximum number of quadratic problems to be solved
        # codeoptions.sqp_nlp.reg_hessian = 5e-9 # increase this if exitflag=-8
        codeoptions.nlp.bfgs_init = 3.0 * np.identity(7)  # initialization of the hessian approximation
        # codeoptions.noVariableElimination = 1

        self.solver = self.model.generate_solver(options=codeoptions)

    def solve(self, x0, target_pixel, target_traj):
        # https://forces.embotech.com/Documentation/solver_options/index.html?highlight=pdip_nlp
        # Set initial condition
        z0i = np.zeros((self.model.nvar, 1))
        z0 = np.transpose(np.tile(z0i, (1, self.model.N)))
        problem = {"x0": z0,
                   "xinit": x0}

        # Set runtime parameters (here, the next N points on the path)
        horizon_inputs = np.zeros((self.model.N, self.model.npar))
        horizon_inputs[:, 3:] = target_pixel
        horizon_inputs[:, :3] = target_traj
        problem["all_parameters"] = np.reshape(horizon_inputs, (-1, 1))

        # Time to solve the NLP!
        output, exitflag, info = self.solver.solve(problem)
        assert exitflag == 1, "failed to solve"

        # Make sure the solver has exited properly.
        # sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n" \
        #                  .format(info.it, info.solvetime))

        # Extract output
        temp = np.zeros((self.model.N, self.model.nvar))
        for i in range(0, self.model.N):
            temp[i, :] = output[f'x{i+1}']
        U_mpc = temp[:, :4]
        X_mpc = temp[:, 4:]
        return X_mpc, U_mpc
