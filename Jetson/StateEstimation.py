import numpy as np
from scipy.spatial.transform import Rotation
import math

def distance(x, y, z, x2, y2, z2):
    return math.sqrt((x - x2)**2 + (y - y2)**2 + (z - z2)**2)

# Kalman Filtering algorithm
class KalmanEstimator(object):

    # Everything is in meters
    def __init__(self, obs_std, acc_std, p_init, v_init, a_init, t_init, invK, threshold=np.Inf):
        # Intialize states
        self.latest_img_frame = None
        self.ltest_drone_state = None

        # Initialize matrices
        self.acc_var = acc_std**2
        self.H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0, 0, 0, 0]])
        self.R = np.array([[obs_std**2, 0, 0],
                           [0, obs_std**2, 0],
                           [0, 0, obs_std**2]])
        self.invK = invK

        # Initialize initial values
        self.last_target_state = (np.array([[p_init[0], p_init[1], p_init[2],
                                            v_init[0], v_init[1], v_init[2],
                                            a_init[0], a_init[1], a_init[2]]]).T, t_init)
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

    # Receive new drone position data
    def receive_drone_state(self, data):
        self.latest_drone_state = data

    # Helper for estimate_3d_pos
    def estimate_ground_plane(self):
        # Assume ground is flat on z = 0 plane and faces upwards
        ground_normal = np.array([0, 0, 1])
        ground_point = np.array([0, 0, 0])
        return ground_normal, ground_point

    # Convert pixel coordinates to 3D absolute coordinates
    def estimate_3d_pos(self, cx, cy, w, h):
        # Calculate pixel data
        pixel = np.array([cx + w / 2,
                          cy + h / 2,
                          1])
        pixel_cam = self.invK @ pixel  # pixel position in camera frame
        pixel_cam = np.array([[0, 0, 1],
                              [-1, 0, 0],
                              [0, -1, 0]]) @ pixel_cam

        # Transform from camera frame to drone frame
        pixel_drone = self.Rdc @ pixel_cam

        # Transform from drone frame to world frame
        T = np.array([self.latest_drone_state.pose.position.x,
                      self.latest_drone_state.pose.position.y,
                      self.latest_drone_state.pose.position.z])
        q = [self.latest_drone_state.pose.orientation.x,
             self.latest_drone_state.pose.orientation.y,
             self.latest_drone_state.pose.orientation.z,
             self.latest_drone_state.pose.orientation.w]
        R = Rotation.from_quat(q).as_matrix()
        pixel_world = R @ pixel_drone + T

        # Calculate ray from camera center to world pixel point
        vec = pixel_world - T
        vec /= np.linalg.norm(vec)

        # Project ray onto ground plane
        ground_normal, ground_point = self.estimate_ground_plane()
        t = (-ground_normal @ (T - ground_point)) / (ground_normal @ vec)
        pred_3d_pos_raw = T + t * vec
        (predx, predy, predz) = pred_3d_pos_raw

        return (predx, predy, predz)

    # Observe a new datapoint from a new frame and use that to update target model 
    def update_target_state(self, cx, cy, w. h, t):
        # Sanity check
        (target_state, prev_t) = self.last_target_state
        del_t = t - prev_t
        if del_t < 0:
            print("Sample time is in the past")
            return None

        # Get 3D absolute position given pixel coordintes and check against threshold
        (x, y, z) = self.estimate_3d_pos(cx, cy, w, h)
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



    def predict_3d_pos(self):
        # read image as RGB format, 8-bit

        if self.img_msg is None:
            print("No image received")
            return None

        if self.drone_pose_msg is None:
            print("No drone pose received")
            return None

        image = self.imgmsg_to_cv2(self.img_msg)

        # detect 2d position of target
        results = detect_object(image)
        if results is None:
            return None

        x, y, w, h, _ = results
        centerX = x + (w / 2)
        centerY = y + (h / 2)

        if DEBUG:
            noise = np.random.random(2) * 10
            centerX += noise[0]
            centerY += noise[1]

            p = np.random.random()
            if p < 0.2:
                print("RANDOM!")
                centerX = np.random.randint(low=0, high=self.camera_info.width)
                centerY = np.random.randint(low=0, high=self.camera_info.height)

        # draw the centerpoint
        image = cv2.circle(image, (int(centerX), int(centerY)), 3, (255, 0, 0), 2)
        cv2.imshow("image", image)
        cv2.waitKey(3)

        pixel = np.array([centerX + self.img_msg.width / 2,
                          centerY + self.img_msg.height / 2,
                          1])
        pixel_cam = self.invK @ pixel  # pixel position in camera frame
        pixel_cam = np.array([[0, 0, 1],
                              [-1, 0, 0],
                              [0, -1, 0]]) @ pixel_cam

        # transform from camera frame to drone frame
        pixel_drone = self.Rdc @ pixel_cam

        # transform from drone frame to world frame
        T = np.array([self.drone_pose_msg.pose.position.x,
                      self.drone_pose_msg.pose.position.y,
                      self.drone_pose_msg.pose.position.z])
        q = [self.drone_pose_msg.pose.orientation.x,
             self.drone_pose_msg.pose.orientation.y,
             self.drone_pose_msg.pose.orientation.z,
             self.drone_pose_msg.pose.orientation.w]
        R = Rotation.from_quat(q).as_matrix()
        pixel_world = R @ pixel_drone + T

        # calculate ray from camera center to world pixel point
        vec = pixel_world - T
        vec /= np.linalg.norm(vec)

        # project ray onto ground plane
        ground_normal, ground_point = self.estimate_ground_plane()
        t = (-ground_normal @ (T - ground_point)) / (ground_normal @ vec)
        pred_3d_pos_raw = T + t * vec
        (predx, predy, predz) = pred_3d_pos_raw

        if self.t0 is None:
            self.t0 = time.time()
            self.estimator.target_state_override(
                x=predx, y=predy, z=predz, vx=0, vy=0, vz=0, ax=0, ay=0, az=0, t=self.t0)
            return pred_3d_pos_raw, None  # first data received, path is useless

        cur_t = time.time()
        self.estimator.receive_sample_absolute(x=predx, y=predy, z=predz, t=cur_t)

        (x, y, z, vx, vy, vz, ax, ay, az) = self.estimator.predict_target_state(cur_t)
        filtered_pred_pos = np.array([x, y, z])
        pred_path = self.predict_3d_path()

        if DEBUG:
            gt_pos = np.array([self.target_gt_pose_msg.pose.position.x,
                               self.target_gt_pose_msg.pose.position.y,
                               self.target_gt_pose_msg.pose.position.z])
            error_pred_kf = np.linalg.norm(filtered_pred_pos - gt_pos)
            error_pred = np.linalg.norm(pred_3d_pos_raw - gt_pos)
            print(f"kf: %.3f, raw: %.3f" % (error_pred_kf, error_pred))

        return filtered_pred_pos, pred_path
