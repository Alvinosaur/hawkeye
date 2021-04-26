#!/usr/bin/env python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import time

import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
# import tf2_ros
# import tf2_geometry_msgs

from Object_detection.objectDetect import detect_object
from state_estimation import StateEstimation
import utils

DEBUG = False


class TargetEstimator3D(object):
    def __init__(self):
        self.camera_info = utils.read_camera_info()
        P = np.array(self.camera_info.P).reshape(3, 4)
        self.K = np.array(self.camera_info.K).reshape(3, 3)
        self.invP = np.linalg.pinv(P)  # pseudo inverse
        self.invK = np.linalg.inv(self.K)
        # Drone -> Camera, Roll pitch yaw, found from the iris_cam sdf file
        self.Rdc = Rotation.from_euler("xyz", [0, 0.785375, 0]).as_matrix()

        self.img_msg = None
        self.target_gt_pose_msg = None
        self.drone_pose_msg = None
        self.t0 = None
        self.N = 10  # prediction horizon length
        self.dt = 0.1

        # Initialize estimator
        ignore_thresh = np.Inf
        obs_std = 0.3  # m
        acc_std = 0.3  # m/s^2
        p_init = (0, 0, 0)
        v_init = (0, 0, 0)
        a_init = (0, 0, 0)
        self.estimator = StateEstimation.KalmanEstimator(obs_std, acc_std, p_init, v_init, a_init, ignore_thresh)

    def image_cb(self, msg: Image):
        self.img_msg = msg

    def target_gt_pose_cb(self, msg: PoseStamped):
        self.target_gt_pose_msg = msg

    def drone_pose_cb(self, msg: PoseStamped):
        self.drone_pose_msg = msg

    def imgmsg_to_cv2(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def estimate_ground_plane(self):
        # assume ground is flat on z = 0 plane and faces upwards
        ground_normal = np.array([0, 0, 1])
        ground_point = np.array([0, 0, 0])
        return ground_normal, ground_point

    def predict_3d_path(self):
        # Expects that predict_3d_pos is called first!
        state_path = [self.estimator.predict_target_state(time.time() + self.dt * i) for i in range(self.N)]
        return utils.create_path(traj=state_path, dt=self.dt, frame="world")

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

    def pos_to_stamped_pose(self, pos):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        x, y, z = pos
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.header.stamp = rospy.Time.now()
        return pose


if __name__ == "__main__":
    rospy.init_node("test_target_estimator_3d")
    target_estimator = TargetEstimator3D()
    image_sub = rospy.Subscriber("/iris_fpv_cam/usb_cam/image_raw", Image, target_estimator.image_cb, queue_size=1)
    target_gt_pose_sub = rospy.Subscriber("/hawkeye/target_pose", PoseStamped, target_estimator.target_gt_pose_cb,
                                          queue_size=1)
    drone_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, target_estimator.drone_pose_cb,
                                      queue_size=1)

    prediction_pub = rospy.Publisher('/hawkeye/target_path_pred', Path, queue_size=10)
    pred_target_pose_pub = rospy.Publisher("/hawkeye/target_pose_pred", PoseStamped, queue_size=10)
    # pred_target_pose_pub_raw = rospy.Publisher("/hawkeye/target_pose_pred_raw", PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # Hz

    while not rospy.is_shutdown():
        predictions = target_estimator.predict_3d_pos()
        if predictions is not None:
            pred_pos, pred_path = predictions
            pred_target_pose_pub.publish(target_estimator.pos_to_stamped_pose(pred_pos))

            if pred_path is not None:
                prediction_pub.publish(pred_path)

        try:  # prevent garbage in console output when thread is killed
            # rate.sleep()
            time.sleep(0.1)
        except rospy.ROSInterruptException:
            pass

    if DEBUG:
        cv2.destroyAllWindows()
