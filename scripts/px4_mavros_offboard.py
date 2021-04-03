#!/usr/bin/env python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String, Header
# import tf_conversions
# import tf2_ros

import time
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation
import math
import numpy as np
import threading
from simple_pid import PID

from inverseDyn import inverse_dyn
from drone_mpc import DroneMPC


# Enums: https://mavlink.io/en/messages/common.html#MAV_FRAME
# SET_POSITION_TARGET_LOCAL_NED: https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
# Sets a desired vehicle position in a local north-east-down coordinate frame
MAV_FRAME_LOCAL_NED = 1  # local x(North), y(East), z(Down)
MAV_FRAME_LOCAL_OFFSET_NED = 7  # local offset (dx, dy, dz) in NED frame
MAV_FRAME_BODY_NED = 8  # local target point in NED x(North), y(East), z(Down)
MAV_FRAME_BODY_OFFSET_NED = 9  # local offset in NED

# SET_ATTITUDE_TARGET: https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
# only option is local
MAV_FRAME_LOCAL_ATT = None


class Px4Controller:
    def __init__(self):
        # User Params
        self.takeoff_height = 3.2
        self.takeoff_height_tol = 0.2

        self.imu_state = None
        self.gps = None
        self.local_pose = None
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

        self.state = None

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        # self.battery_sub = rospy.Subscriber("/mavros/battery", Imu, self.battery_callback)

        '''
        ros publishers
        '''
        self.pos_control_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.att_control_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        # FLAT_STATES = 7
        # FLAT_CTRLS = 4
        # A = np.zeros((FLAT_STATES, FLAT_STATES))
        # A[0:3, 3:6] = np.eye(3)
        # B = np.zeros((FLAT_STATES, FLAT_CTRLS))
        # B[3:, :] = np.eye(4)
        # Gff = np.array([[0, 0, g, 0]]).T  # gravity compensation
        # Q = np.diag([20, 20, 20, 0.1, 0.1, 0.1, 1])
        # R = np.diag([.1, .1, .1, 10])
        # S = Q * 10
        #
        # # u_lower = np.array([[-1, -1]]).T
        # # u_upper = np.array([[3, 3]]).T
        # # u_constraints = np.hstack([u_lower, u_upper])
        #
        # self.dt = 0.1
        # self.N = 10  # 1 second horizon
        #
        # x_lower = np.array([[-1, -1, -1, -1]]).T
        # x_upper = np.array([[5, 6, 3, 3]]).T
        # x_constraints = np.hstack([x_lower, x_upper])
        # self.mpc = DroneMPC(A, B, Q, R, S, N=self.N, dt=self.dt, x_constraints=x_constraints)
        #
        # self.pid_vx = PID(Kp=7, Ki=0, Kd=1, setpoint=0)
        # self.pid_vy = PID(Kp=7, Ki=0, Kd=1, setpoint=0)
        # self.pid_vz = PID(Kp=7, Ki=0, Kd=1, setpoint=0)
        #
        # print("Px4 Controller Initialized!")

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

    # def generate_action(self, ref_pos_traj):
    #     # convert position trajectory to state trajectory
    #
    #     dphi = pid_phi(phi - phid)
    #     xref = np.array([[
    #         pos_g[0], pos_g[1], pos_g[2],
    #         vel_g[0], vel_g[1], vel_g[2],
    #         psis[0]]]).T
    #
    #     [thrustd, phid, thetad, psid] = inverse_dyn(rot, x.flatten(), u, m)
    #     u = u_mpc[:, 0].flatten() + Gff.flatten()

    def reached_target(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False

    def local_pose_callback(self, msg):
        self.local_pose = msg

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

    def FLU2ENU(self, msg):
        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(
            self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(
            self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z

    def set_target_position_callback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg)

            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)

    '''
     Receive A Custom Activity
     '''

    def custom_activity_callback(self, msg):
        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         0.1,
                                                         self.current_heading)

        elif msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")

    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_deg)

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
        rospy.init_node("offboard_node")
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
            desired_pos = self.construct_target(x=0, y=0, z=8, q=q)
            self.pos_control_pub.publish(desired_pos)
            if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):
                if self.disarm():
                    self.state = "DISARMED"

            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == '__main__':
    con = Px4Controller()
    con.start()
