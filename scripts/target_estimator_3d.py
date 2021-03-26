#!/usr/bin/env python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, Float64, String, Header
import time
from pyquaternion import Quaternion
import math
import numpy as np
from geometry_msgs.msg import PoseStamped


class TargetEstimator3D(object):
    def __init__(self):
        self.camera_info = self.read_camera_info()
        self.img_msg = None
        self.target_gt_pose_msg = None
        self.drone_pose_msg = None

    def read_camera_info(self) -> CameraInfo:
        return rospy.wait_for_message("/iris_fpv_cam/usb_cam/camera_info", CameraInfo)

    def image_cb(self, msg):
        self.img_msg = msg

    def target_gt_pose_cb(self, msg):
        self.target_gt_pose_msg = msg

    def drone_pose_cb(self, msg):
        self.drone_pose_msg = msg

    def predict_3d_pose(self):
        # Calculate 3d target pose in camera frame (from 2d image frame)
        P = np.array(self.camera_info.P).reshape(3, 4)
        invP = np.linalg.pinv(P)  # pseudo inverse
        
        pixel = [x, y, 1.]  # relative to center, append z = 1
        pixel_world = np.dot(invP, pixel)


if __name__ == "__main__":
    rospy.init_node("test_target_estimator_3d")
    target_estimator = TargetEstimator3D()
    image_sub = rospy.Subscriber("/iris_fpv_cam/usb_cam/image_raw", Image, target_estimator.image_cb, queue_size=1)
    target_gt_pose_sub = rospy.Subscriber("/hawkeye/target_pose", PoseStamped, target_estimator.target_gt_pose_cb,
                                          queue_size=1)
    drone_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, target_estimator.drone_pose_cb,
                                      queue_size=1)

    pred_target_pose_pub = rospy.Publisher("/hawkeye/target_pose_pred", PoseStamped)
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():

        try:  # prevent garbage in console output when thread is killed
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
