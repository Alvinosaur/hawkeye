from roslaunch.parent import ROSLaunchParent
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import os
import signal
from objectDetect import detect_object

# Launch roscore
parent = ROSLaunchParent("TX1", [], is_core = True)
parent.start()
print("\n--------------------------------------\n\n")

bridge = CvBridge()
#cv2.startWindowThread()
#cv2.namedWindow("Tracking")

# Function which takes in an image and runs our processing
def process(data):
    cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2_img = cv2.rotate(cv2_img, cv2.ROTATE_180)
    try:
        (x, y, w, h, mask) = detect_object(cv2_img)
        res = cv2.rectangle(cv2_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.imshow("Tracking", res)
        cv2.waitKey(1)
    except:
        cv2.imshow("Tracking", cv2_img)
        cv2.waitKey(1)


def shutdown_ros():
    cv2.destroyAllWindows()
    parent.shutdown()
    os._exit(0)

rospy.init_node("img_proc")
rospy.on_shutdown(shutdown_ros)
sb = rospy.Subscriber('image', Image, process, queue_size = 10)
pb = rospy.Publisher('events', String, queue_size = 10)
rospy.Rate(10)
while not rospy.is_shutdown():
    x = raw_input()
    pb.publish(x)
    if (x == "stop"):
        cv2.destroyAllWindows()
    elif (x == "kill"):
        os.kill(os.getpid(), signal.SIGINT)

