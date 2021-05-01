#!/usr/bin/env python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget, Thrust
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
import threading


class Px4Controller:
    def __init__(self):
        # Takeoff
        self.takeoff_height = 8
        self.takeoff_height_tol = 0.2
        self.takeoff_q = Rotation.from_euler('zyx', [45, 0, 0], degrees=True).as_quat()

        # motion planner
        self.threshold_timeout = 0.5
        self.return_home_timeout = 3
        self.N = 4
        self.dt = 0.5
        # self.drone_mpc = DroneMPC(N=self.N, dt=self.dt, load_solver=True)
        self.drone_mpc = DroneMPC(N=self.N)
        self.t_offset = 0
        self.t_solved = time.time() + self.t_offset
        self.solver_thread = None

        self.local_pose = None
        self.local_q = None
        self.local_vel = None

        self.mavros_state = None
        self.arm_state = False
        self.flight_mode = "Land"

        self.target_path = None
        self.X_mpc = None
        self.U_mpc = None
        self.X_all = []
        self.target_all = []

        rospy.init_node("offboard_node")
        # Subscribers
        self.local_vel_sub = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped,
                                              self.local_vel_callback,
                                              queue_size=1)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback,
                                               queue_size=1)
        self.target_path_sub = rospy.Subscriber("/hawkeye/target_path_pred", Path, self.target_path_cb,
                                                queue_size=1)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback, queue_size=1)

        # Publishers
        self.pos_control_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        # self.att_control_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        self.att_control_pub = rospy.Publisher('mavros/setpoint_attitude/attitude', PoseStamped, queue_size=10)
        self.thrust_control_pub = rospy.Publisher('mavros/setpoint_attitude/thrust', Thrust, queue_size=10)
        self.output_path_pub = rospy.Publisher('/hawkeye/drone_path', Path, queue_size=10)

        # Services
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # Drone camera parameters
        camera_info = utils.read_camera_info()
        K = np.array(camera_info.K).reshape(3, 3)
        invK = np.linalg.inv(K)
        # Drone -> Camera, Roll pitch yaw, found from the iris_cam sdf file
        Rdc = Rotation.from_euler("xyz", [0, 0.785375, 0]).as_matrix()

        # we want target to lie in center of image
        target_pix_x = camera_info.width / 2
        target_pix_y = camera_info.height / 2
        target_pixel = np.array([target_pix_x + camera_info.width / 2,
                                 target_pix_y + camera_info.height / 2,
                                 1])
        pixel_cam = invK @ target_pixel  # pixel position in camera frame
        pixel_cam = np.array([[0, 0, 1],
                              [-1, 0, 0],
                              [0, -1, 0]]) @ pixel_cam

        # transform from camera frame to drone frame
        self.pixel_drone = Rdc @ pixel_cam

    @staticmethod
    def quat_from_pose(pose_msg):
        return np.array([pose_msg.pose.orientation.x,
                         pose_msg.pose.orientation.y,
                         pose_msg.pose.orientation.z,
                         pose_msg.pose.orientation.w])

    def check_connection(self):
        for i in range(10):
            if self.local_pose is not None and self.local_vel is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        return self.local_pose is not None and self.local_vel is not None

    def takeoff(self):
        max_count = 100  # 200 * 0.2 sec = 20 sec for takeoff
        takeoff_pos = self.construct_pose_target(x=0, y=0, z=self.takeoff_height, q=self.takeoff_q)
        count = 0
        while not self.takeoff_detection() and count < max_count:
            self.pos_control_pub.publish(takeoff_pos)
            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)
            print("Height: %.3f" % self.local_pose.pose.position.z)
            count += 1

        return count < max_count

    def check_target_tracked(self):
        if self.target_path is None: return False, np.Inf
        t = rospy.get_rostime().to_sec()
        prev_t = self.target_path.header.stamp.to_sec()
        print(t, prev_t, t - prev_t < self.threshold_timeout)
        return t - prev_t < self.threshold_timeout, t - prev_t

    def construct_pose_target(self, x, y, z, q=np.array([0, 0, 0, 1])):
        target_raw_pose = PoseStamped()
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.pose.position.x = x
        target_raw_pose.pose.position.y = y
        target_raw_pose.pose.position.z = z

        target_raw_pose.pose.orientation.x = q[0]
        target_raw_pose.pose.orientation.y = q[1]
        target_raw_pose.pose.orientation.z = q[2]
        target_raw_pose.pose.orientation.w = q[3]

        return target_raw_pose

    def generate_action(self):
        if (self.local_pose is None or
                self.local_vel is None or
                self.target_path is None):
            return None

        target_traj = np.vstack([(pose.pose.position.x,
                                  pose.pose.position.y,
                                  pose.pose.position.z) for pose in self.target_path.poses])

        q = self.local_q
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

        self.X_all.append(x)
        self.target_all.append(target_traj)

        start_time = time.time()
        try:
            self.X_mpc, self.U_mpc = self.drone_mpc.solve(
                x0=x, target_pixel=self.pixel_drone,
                target_traj=target_traj, prev_X=self.X_mpc)
        except Exception as e:
            print("Failed to solve due to: %s" % e)
            return None

        print("Time to solve: %.1f" % (time.time() - start_time))
        self.t_solved = time.time() + self.t_offset

    # def generate_action_v2(self):
    #

    def use_prev_traj(self, path_index):
        x, y, z = self.X_mpc[path_index, :3]
        # thrust, phi, theta, psi = self.drone_mpc.inverse_dyn(q=self.local_q, x_ref=self.X_mpc[1], u=self.U_mpc[path_index])
        yaw = self.X_mpc[path_index, -1] % (2 * math.pi)
        target_q = Rotation.from_euler("XYZ", [0, 0, yaw]).as_quat()
        pose_msg = self.construct_pose_target(x=x, y=y, z=z, q=target_q)

        return pose_msg

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
        self.local_q = self.quat_from_pose(msg)

    def local_vel_callback(self, msg):
        self.local_vel = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode

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
        # DO NOT USE, it's safer to manually switch on offboard mode
        if self.flightModeService(custom_mode=State.MODE_PX4_OFFBOARD):
            return True
        else:
            print("Vehicle Offboard failed")
            return False

    def takeoff_detection(self):
        reached_takeoff_height = (
                (abs(self.local_pose.pose.position.z - self.takeoff_height) < self.takeoff_height_tol) or
                self.local_pose.pose.position.z > self.takeoff_height)
        cur_q = self.local_q
        reached_takeoff_ori = np.isclose(abs(self.takeoff_q @ cur_q), 1.0, atol=1e-3)
        return reached_takeoff_height and reached_takeoff_ori and self.arm_state

    def start(self):
        if not self.check_connection():
            print("Failed to connect!")
            return

        # takeoff to reach desired tracking height
        if not self.takeoff():
            print("Failed to takeoff!")
            return
        print("Successful Takeoff!")
        self.t0 = rospy.get_rostime().to_sec()

        rate = rospy.Rate(int(1 / self.dt))  # Hz

        while self.arm_state and not rospy.is_shutdown():
            if self.mavros_state == State.MODE_PX4_OFFBOARD:
                targed_tracked, time_since_detected = self.check_target_tracked()
                if (self.solver_thread is None or not self.solver_thread.is_alive()) and targed_tracked:
                    print("Generating action!")
                    # Spawn a process to run this independently:
                    self.solver_thread = threading.Thread(target=self.generate_action)
                    self.solver_thread.start()

                path_index = int(((time.time() - self.t_solved) / self.dt))
                print(path_index)
                path_index = min(max(0, path_index), self.N-1)
                print("Path index: %d" % path_index)
                print("time_since_detected", time_since_detected)
                if time_since_detected > self.return_home_timeout or self.X_mpc is None:
                    print("Staying at origin!")
                    desired_pos = self.construct_pose_target(
                        x=0,
                        y=0,
                        z=self.takeoff_height, q=self.takeoff_q)
                else:
                    print("Using prev path!")
                    desired_pos = self.use_prev_traj(path_index)

                if self.X_mpc is not None:
                    pos_traj = [self.X_mpc[i, :3].tolist() for i in range(self.N)]
                    self.output_path_pub.publish(utils.create_path(traj=pos_traj, dt=self.dt, frame="world"))

                self.pos_control_pub.publish(desired_pos)

            # if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):
            #     if self.disarm():
            #         self.state = "DISARMED"

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':
    con = Px4Controller()
    con.start()
