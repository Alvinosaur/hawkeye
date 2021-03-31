import numpy as np
from scipy.interpolate import UnivariateSpline
import math

def distance(x, y, z, x2, y2, z2):
    return math.sqrt((x - x2)**2 + (y - y2)**2 + (z - z2)**2)

# Kalman Filtering algorithm
class KalmanEstimator(object):

    # Everything is in meters
    def __init__(self, threshold, obs_std, acc_std, p_init, v_init, a_init):
        # Initialize matrices
        self.acc_var = acc_std**2
        self.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0]])
        self.R = np.array([[obs_std**2, 0, 0],
                           [0, obs_std**2, 0],
                           [0, 0, obs_std**2]])

        # Initialize initial values
        self.last_drone_state = (np.array([[p_init[0], p_init[1], p_init[2],
                                            v_init[0], v_init[1], v_init[2],
                                            a_init[0], a_init[1], a_init[2]]]).T, 0)
        self.last_target_state = (np.array([[p_init[0], p_init[1], p_init[2],
                                            v_init[0], v_init[1], v_init[2],
                                            a_init[0], a_init[1], a_init[2]]]).T, 0)
        self.P = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 0]])

        # Threshold for bogus values
        self.threshold = threshold

    # Generate F matrix
    def F(self, del_T):
        del_2 = del_T**2
        return np.array([[1, 0, 0, del_T, 0, 0, del_2, 0, 0],
                         [0, 1, 0, 0, del_T, 0, 0, del_2, 0],
                         [0, 0, 1, 0, 0, del_T, 0, 0, del_2],
                         [0, 0, 0, 1, 0, 0, del_T, 0, 0],
                         [0, 0, 0, 0, 1, 0, 0, del_T, 0],
                         [0, 0, 0, 0, 0, 1, 0, 0, del_T],
                         [0, 0, 0, 0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 0, 0, 0, 1]])

    # Generate Q matrix
    def Q(self, del_T):
        del_2 = del_T**2
        del_3 = del_T**3
        del_4 = del_T**4
        return np.array([[del_4*self.acc_var, 0, 0, del_3*self.acc_var, 0, 0, del_2*self.acc_var, 0, 0],
                         [0, del_4*self.acc_var, 0, 0, del_3*self.acc_var, 0, 0, del_2*self.acc_var, 0],
                         [0, 0, del_4*self.acc_var, 0, 0, del_3*self.acc_var, 0, 0, del_2*self.acc_var],
                         [del_3*self.acc_var, 0, 0, del_2*self.acc_var, 0, 0, del_T*self.acc_var, 0, 0],
                         [0, del_3*self.acc_var, 0, 0, del_2*self.acc_var, 0, 0, del_T*self.acc_var, 0],
                         [0, 0, del_3*self.acc_var, 0, 0, del_2*self.acc_var, 0, 0, del_T*self.acc_var],
                         [del_2*self.acc_var, 0, 0, del_T*self.acc_var, 0, 0, self.acc_var, 0, 0],
                         [0, del_2*self.acc_var, 0, 0, del_T*self.acc_var, 0, 0, self.acc_var, 0],
                         [0, 0, del_2*self.acc_var, 0, 0, del_T*self.acc_var, 0, 0, self.acc_var]])

    # Observe a new datapoint where x, y, z are absolute coordinates
    def receive_sample_absolute(self, x, y, z, t):
        # See if current target position falls within threshold
        (target_state, prev_t) = self.last_target_state
        del_t = t - prev_t
        if del_t < 0:
            print("Sample time is in the past")
            return 1/0
        F = self.F(del_t)
        pred_hat = F @ target_state
        if distance(x, y, z, pred_hat[0][0], pred_hat[1][0], pred_hat[2][0]) > self.threshold:
            return None

        # Predict next target state
        Q = self.Q(del_t)
        covar_hat = (F @ self.P @ F.T) + Q
        K = (covar_hat @ self.H.T) @ np.linalg.inv((self.H @ covar_hat @ self.H.T) + self.R)
        cur_target_state = pred_hat + (K @ (np.array([[x, y, z]]).T - (self.H @ pred_hat)))
        self.P = covar_hat - (K @ self.H @ covar_hat)
        self.last_target_state = (cur_target_state, t)

    # Observe a new datapoint where dx, dy, dz are relative to drone position
    # REQUIRES THAT receive_drone_state WAS CALLED SOMETIME IN THE NEAR PAST
    def receive_sample_relative(self, dx, dy, dz, t):
        # Update current drone position and get absolute coords of sample
        (drone_state, prev_t) = self.last_drone_state
        del_t = t - prev_t
        if del_t < 0:
            print("Sample time is in the past")
            return 1/0
        F = self.F(del_t)
        cur_drone_state = F @ drone_state
        (drone_x, drone_y, drone_z) = (cur_drone_state[0][0], cur_drone_state[1][0], cur_drone_state[2][0])
        x = drone_x + dx
        y = drone_y + dy
        z = drone_z + dz

        # See if current target position falls within threshold
        (target_state, prev_t) = self.last_target_state
        del_t = t - prev_t
        if del_t < 0:
            print("Sample time is in the past")
            return 1/0
        F = self.F(del_t)
        pred_hat = F @ target_state
        if distance(x, y, z, pred_hat[0][0], pred_hat[1][0], pred_hat[2][0]) > self.threshold:
            return None

        # Predict next target state
        Q = self.Q(del_t)
        covar_hat = (F @ self.P @ F.T) + Q
        K = (covar_hat @ self.H.T) @ np.linalg.inv((self.H @ covar_hat @ self.H.T) + self.R)
        cur_target_state = pred_hat + (K @ (np.array([[x, y, z]]).T - (self.H @ pred_hat)))
        self.P = covar_hat - (K @ self.H @ covar_hat)
        self.last_target_state = (cur_target_state, t)

    # Manually override the current target state
    def target_state_override(self, x, y, z, vx, vy, vz, ax, ay, az, t):
         self.last_target_state = (np.array([[x, y, z, vx, vy, vz, ax, ay, az]]).T, t)

    # Observe new drone state
    def receive_drone_state(self, x, y, z, vx, vy, vz, ax, ay, az, t):
        (drone_state, prev_t) = self.last_drone_state
        if t < prev_t:
            print("Sample time is in the past")
            return 1/0
        self.last_drone_state = (np.array([[x, y, z, vx, vy, vz, ax, ay, az]]).T, t)

    # Using current motion model, output target state at time t
    def predict_target_state(self, t):
        (target_state, prev_t) = self.last_target_state
        del_t = t - prev_t
        if del_t < 0:
            print("Prediction time is in the past")
            return 1/0
        pred_hat = self.F(del_t) @ target_state
        return (pred_hat[0][0], pred_hat[1][0], pred_hat[2][0], 
                pred_hat[3][0], pred_hat[4][0], pred_hat[5][0], 
                pred_hat[6][0], pred_hat[7][0], pred_hat[8][0])
