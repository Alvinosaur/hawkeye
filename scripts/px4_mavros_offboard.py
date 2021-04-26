#!/usr/bin/env python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String, Header

import time
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation
import math
import numpy as np

from drone_mpc import DroneMPC
import utils


class Px4Controller:
    def __init__(self):
        # User Params
        self.takeoff_height = 3.2
        self.takeoff_height_tol = 0.2

        self.imu_state = None
        self.gps = None
        self.local_pose = None
        self.local_vel = None
        self.current_state = None
        self.battery_state = None
        self.current_heading = None
        self.local_enu_position = None

        self.cur_target_pose = None
        self.global_target = None

        self.received_new_task = False
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.target_path = None
        self.state = None
        self.X_mpc = None
        self.U_mpc = None

        # motion planner
        self.N = 5
        self.dt = 0.1
        self.drone_mpc = DroneMPC(N=self.N)

        rospy.init_node("offboard_node")
        '''
        ros subscribers
        '''
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped,
                                              self.local_vel_callback,
                                              queue_size=1)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback,
                                               queue_size=1)
        self.target_path_sub = rospy.Subscriber("/hawkeye/target_path_pred", Path, self.target_path_cb,
                                                queue_size=1)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback, queue_size=1)
        # self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback, queue_size=1)
        # self.battery_sub = rospy.Subscriber("/mavros/battery", Imu, self.battery_callback)

        '''
        ros publishers
        '''
        self.pos_control_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.att_control_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.output_path_pub = rospy.Publisher('/hawkeye/drone_path', Path, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Drone camera parameters
        # camera_info = utils.read_camera_info()
        # P = np.array(camera_info.P).reshape(3, 4)
        # K = np.array(camera_info.K).reshape(3, 3)
        # invP = np.linalg.pinv(P)  # pseudo inverse
        # invK = np.linalg.inv(K)
        # # Drone -> Camera, Roll pitch yaw, found from the iris_cam sdf file
        # Rdc = Rotation.from_euler("xyz", [0, 0.785375, 0]).as_matrix()
        # 
        # # we want target to lie in center of image
        # target_pix_x = camera_info.width / 2
        # target_pix_y = camera_info.height / 2
        # target_pixel = np.array([target_pix_x + camera_info.width / 2,
        #                          target_pix_y + camera_info.height / 2,
        #                          1])
        # pixel_cam = invK @ target_pixel  # pixel position in camera frame
        # pixel_cam = np.array([[0, 0, 1],
        #                       [-1, 0, 0],
        #                       [0, -1, 0]]) @ pixel_cam
        # 
        # # transform from camera frame to drone frame
        # self.pixel_drone = Rdc @ pixel_cam

    def check_connection(self):
        for i in range(10):
            if self.local_pose is not None and self.imu_state is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        return self.local_pose is not None and self.imu_state is not None

    def takeoff(self):
        max_count = 100  # 200 * 0.2 sec = 20 sec for takeoff
        takeoff_pos = self.construct_target(x=0, y=0, z=self.takeoff_height)
        count = 0
        while not self.takeoff_detection() and count < max_count:
            self.pos_control_pub.publish(takeoff_pos)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)
            print("Height: %.3f" % self.local_pose.pose.position.z)
            count += 1

        return count < max_count

    def construct_target(self, x, y, z, q=np.array([0, 0, 0, 1])):
        target_raw_pose = PoseStamped()
        target_raw_pose.header.stamp = rospy.Time.now()

        # target_raw_pose.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        target_raw_pose.pose.position.x = x
        target_raw_pose.pose.position.y = y
        target_raw_pose.pose.position.z = z

        # target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
        #                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
        #                             + PositionTarget.FORCE

        target_raw_pose.pose.orientation.x = q[0]
        target_raw_pose.pose.orientation.y = q[1]
        target_raw_pose.pose.orientation.z = q[2]
        target_raw_pose.pose.orientation.w = q[3]
        # target_raw_pose.yaw = yaw
        # target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose

    def generate_action(self):
        if self.local_pose is None or self.local_vel is None:
            return None
        assert self.target_path is not None
        target_traj = [(pose.pose.position.x,
                        pose.pose.position.y,
                        pose.pose.position.z) for pose in self.target_path.poses]

        q = [self.local_pose.pose.orientation.x,
             self.local_pose.pose.orientation.y,
             self.local_pose.pose.orientation.z,
             self.local_pose.pose.orientation.w]
        roll, pitch, yaw = Rotation.from_quat(q).as_euler("XYZ")

        x = np.array([
            self.local_pose.pose.position.x,
            self.local_pose.pose.position.y,
            self.local_pose.pose.position.z,
            self.local_vel.twist.linear.x,
            self.local_vel.twist.linear.y,
            self.local_vel.twist.linear.z,
            yaw
        ])

        np.savez("drone_target_sample", target_traj=target_traj,
                 drone_x=x, drone_q=q, pixel_drone=self.pixel_drone)
        exit(1)


        user_sq_dist = 0.5
        X_mpc, self.U_mpc = self.drone_mpc.solve(
            x0=x, u0=None, target_pixel=self.pixel_drone,
            phi0=roll, target_traj=target_traj, user_sq_dist=user_sq_dist)

        pos_traj = [X_mpc[i, :3].tolist() for i in range(self.N)]
        self.output_path_pub.publish(utils.create_path(traj=pos_traj, dt=self.dt, frame="world"))

    def reached_target(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False

    def target_path_cb(self, msg):
        self.target_path = msg

    def local_pose_callback(self, msg):
        self.local_pose = msg

    def local_vel_callback(self, msg):
        self.local_vel = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

    def imu_callback(self, msg):
        self.imu_state = msg
        self.current_heading = self.q2yaw(self.imu_state.orientation)
        self.received_imu = True

    def battery_callback(self, msg):
        self.battery_state = msg

    def gps_callback(self, msg):
        self.gps = msg

    '''
    return yaw from current IMU
    '''

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False

    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False

    def hover(self):
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)

    def takeoff_detection(self):
        reached_takeoff_height = abs(self.local_pose.pose.position.z - self.takeoff_height) < self.takeoff_height_tol
        return reached_takeoff_height and self.offboard_state and self.arm_state

    def start(self):
        if not self.check_connection():
            print("Failed to connect!")
            return

        # takeoff to reach desired tracking height
        if not self.takeoff():
            print("Failed to takeoff!")
            return

        rate = rospy.Rate(10)  # Hz
        att = AttitudeTarget()
        att.body_rate = Vector3()
        att.header = Header()
        att.header.frame_id = "base_footprint"
        att.orientation = self.local_pose.pose.orientation

        att.body_rate.x = 0
        att.body_rate.y = 0
        att.body_rate.z = 0.1
        att.thrust = 0.7
        att.type_mask = 3  # ignore roll and pitch rate

        q = Rotation.from_euler('zyx', [45, 0, 0], degrees=True).as_quat()

        while self.arm_state and self.offboard_state and not rospy.is_shutdown():
            att.header.stamp = rospy.Time.now()
            desired_pos = self.construct_target(x=0, y=0, z=self.takeoff_height, q=q)
            self.pos_control_pub.publish(desired_pos)
            print(self.local_pose.pose.position.z)
            # if np.isclose(self.local_pose.pose.position.z, self.takeoff_height, atol=1e-2):
            #     if self.target_path is not None:
            #         self.generate_action()
            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):
                if self.disarm():
                    self.state = "DISARMED"

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def start_debug(self):
        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            if self.target_path is not None:
                self.generate_action()

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':
    con = Px4Controller()
    con.start()
