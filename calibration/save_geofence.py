#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import numpy as np
import matplotlib.pyplot as plt


class SaveGeofence(object):
    def __init__(self):
        self.local_xs = []
        self.local_ys = []
        self.local_zs = []
        self.global_xs = []
        self.global_ys = []
        self.global_zs = []

    def local_pose_callback(self, msg):
        self.local_xs.append(msg.pose.position.x)
        self.local_ys.append(msg.pose.position.y)
        self.local_zs.append(msg.pose.position.z)

    def global_pose_callback(self, msg):
        self.global_xs.append(msg.pose.pose.position.x)
        self.global_ys.append(msg.pose.pose.position.y)
        self.global_zs.append(msg.pose.pose.position.z)

    # def local_pose_callback(self, msg):
    #     self.local_pose = msg
    #     self.xs.append(self.local_pose.pose.position.x)
    #     self.xs.append(self.local_pose.pose.position.x)
    #     self.xs.append(self.local_pose.pose.position.x)


if __name__ == '__main__':
    load = True

    if not load:
        rospy.init_node("save_geofence")

        geofence_saver = SaveGeofence()

        local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped,
                                          geofence_saver.local_pose_callback,
                                          queue_size=1)
        global_pose_sub = rospy.Subscriber("/mavros/global_position/local", Odometry,
                                           geofence_saver.global_pose_callback,
                                           queue_size=1)

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

        np.savez("geofence_map",
                 local_x=geofence_saver.local_xs,
                 local_y=geofence_saver.local_ys,
                 local_z=geofence_saver.local_zs,
                 global_x=geofence_saver.global_xs,
                 global_y=geofence_saver.global_ys,
                 global_z=geofence_saver.global_zs)


    else:
        data = np.load("geofence_map.npz")
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(data["global_x"], data["global_y"], data["global_z"], "b", label="global")
        ax.scatter(data["local_x"], data["local_y"], data["local_z"], "r", label="local")

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')

        plt.legend()
        plt.show()
