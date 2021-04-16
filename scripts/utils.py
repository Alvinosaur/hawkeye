import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import CameraInfo


def create_path(traj, dt, frame):
    poses = []
    cur_tstamp = rospy.Time.now()
    for i in range(len(traj)):
        tstamp = cur_tstamp + rospy.Duration(dt * i)
        p = PoseStamped()

        p.pose.position = Point(*traj[i][:3])

        p.header.frame_id = "world"
        p.header.stamp = tstamp
        poses.append(p)

    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = cur_tstamp
    path.poses = poses
    return path


def read_camera_info() -> CameraInfo:
    img_info = None
    max_tries = 100
    count = 0
    while img_info is None and count <= max_tries:
        count += 1
        img_info = rospy.wait_for_message("/iris_fpv_cam/usb_cam/camera_info", CameraInfo)

    return img_info
