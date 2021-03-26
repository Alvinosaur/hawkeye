import numpy as np
from scipy.interpolate import UnivariateSpline
import math

def distance(x, y, x2, y2):
    return math.sqrt((x - x2)**2 + (y - y2)**2)

# Kalman Filtering algorithm
class KalmanEstimator(object):

    def __init__(self, fps, obs_std, acc_std, p_init, v_init, a_init):
        # Initialize matrices
        del_T = 1/fps
        del_2 = del_T**2
        del_3 = del_T**3
        del_4 = del_T**4
        acc_var = acc_std**2
        self.F = np.array([[1, 0, del_T, 0, del_2, 0],
                           [0, 1, 0, del_T, 0, del_2],
                           [0, 0, 1, 0, del_T, 0],
                           [0, 0, 0, 1, 0, del_T],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])
        self.Q = np.array([[del_4*acc_var, 0, del_3*acc_var, 0, del_2*acc_var, 0],
                           [0, del_4*acc_var, 0, del_3*acc_var, 0, del_2*acc_var],
                           [del_3*acc_var, 0, del_2*acc_var, 0, del_T*acc_var, 0],
                           [0, del_3*acc_var, 0, del_2*acc_var, 0, del_T*acc_var],
                           [del_2*acc_var, 0, del_T*acc_var, 0, acc_var, 0],
                           [0, del_2*acc_var, 0, del_T*acc_var, 0, acc_var]])
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0]])
        self.R = np.array([[obs_std**2, 0],
                           [0, obs_std**2]])

        # Initialize initial values
        self.predicted = []
        self.predicted.append(np.array([[p_init[0], p_init[1], v_init[0], v_init[1], a_init[0], a_init[1]]]).T)
        self.covariance = []
        self.covariance.append(np.array([[0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0],
                                         [0, 0, 0, 0, 0, 0]]))
        self.observations = []
        self.new_observation = False

    # Observe a new datapoint
    def observe(self, x, y):
        self.observations.append(np.array([[x, y]]).T)
        self.new_observation = True

    # Predicts state k steps into the future
    def predict(self, k = 1):
        if k <= 0:
            print("k must be > 0")
            return None

        if self.new_observation:
            pred_hat = np.matmul(self.F, self.predicted[-1])
            covar_hat = (self.F @ self.covariance[-1] @ self.F.T) + self.Q
            K = (covar_hat @ self.H.T) @ np.linalg.inv((self.H @ covar_hat @ self.H.T) + self.R)
            pred_new = pred_hat + (K @ (self.observations[-1] - (self.H @ pred_hat)))
            covar_new = covar_hat - (K @ self.H @ covar_hat)

            self.predicted.append(pred_new)
            self.covariance.append(covar_new)
            self.new_observation = False

            return(pred_new[0][0], pred_new[1][0])

        else:
            return(self.predicted[-1][0][0], self.predicted[-1][1][0])

    def get_vel(self):
        return (self.predicted[-1][2][0], self.predicted[-1][3][0])




