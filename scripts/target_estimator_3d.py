#!/usr/bin/env python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import time

import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
# import tf2_ros
# import tf2_geometry_msgs

from Object_detection.objectDetect import detect_object

DEBUG = True


class TargetEstimator3D(object):
    def __init__(self):
        self.camera_info = self.read_camera_info()
        # print(self.camera_info.K)
        P = np.array(self.camera_info.P).reshape(3, 4)
        self.K = np.array(self.camera_info.K).reshape(3, 3)
        self.invP = np.linalg.pinv(P)  # pseudo inverse
        self.invK = np.linalg.inv(self.K)
        # Drone -> Camera, Roll pitch yaw, found from the iris_cam sdf file
        self.Rdc = Rotation.from_euler("xyz", [0, 0.785375, 0]).as_matrix()
        # self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))  # tf buffer length
        # tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.img_msg = None
        self.target_gt_pose_msg = None
        self.drone_pose_msg = None

    def read_camera_info(self) -> CameraInfo:
        img_info = None
        max_tries = 100
        count = 0
        while img_info is None and count <= max_tries:
            count += 1
            img_info = rospy.wait_for_message("/iris_fpv_cam/usb_cam/camera_info", CameraInfo)

        return img_info

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
        # return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

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

        # draw the centerpoint
        image = cv2.circle(image, (int(centerX), int(centerY)), 3, (255, 0, 0), 2)
        cv2.imshow("image", image)
        cv2.waitKey(3)

        pixel = np.array([centerX + self.img_msg.width / 2,
                          centerY + self.img_msg.height / 2,
                          1])
        pixel_cam = self.invK @ pixel  # pixel position in camera frame
        pixel_cam = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]) @ pixel_cam

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

        vec = pixel_world - T
        vec /= np.linalg.norm(vec)
        ground_normal = np.array([0, 0, 1])
        ground_point = np.array([0, 0, self.target_gt_pose_msg.pose.position.z / 2])
        t = (-ground_normal @ (T - ground_point)) / (ground_normal @ vec)
        ground_plane_point = T + t * vec
        # true_position = [self.target_gt_pose_msg.pose.position.x,
        #                  self.target_gt_pose_msg.pose.position.y,
        #                  self.target_gt_pose_msg.pose.position.z]
        # print(f"true: {true_position}", f"pred: {ground_plane_point}", pixel_world)
        # import ipdb
        # ipdb.set_trace()
        ground_plane_point[2] = self.target_gt_pose_msg.pose.position.z
        return ground_plane_point

    def pos_to_stamped_pose(self, pos):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        x, y, z = pos
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # pose.pose.orientation = [0, 0, 0, 1]
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

    pred_target_pose_pub = rospy.Publisher("/hawkeye/target_pose_pred", PoseStamped, queue_size=10)
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        T = target_estimator.predict_3d_pos()
        if T is not None:
            pred_target_pose_pub.publish(target_estimator.pos_to_stamped_pose(T))

        try:  # prevent garbage in console output when thread is killed
            # rate.sleep()
            time.sleep(0.1)
        except rospy.ROSInterruptException:
            pass

    if DEBUG:
        cv2.destroyAllWindows()
