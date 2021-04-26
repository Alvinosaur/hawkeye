from roslaunch.parent import ROSLaunchParent
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
import numpy as np
import os
import sys
import signal
import zc.lockfile
from objectDetect import detect_object

# Initialization
lock = zc.lockfile.LockFile('lock')     # Ensures two instances of program can't be run simultaneously
f = open("/home/siddesh/hawkeye/Jetson/px4_log.txt", "w", buffering=0)

# Launch Roscore
parent = ROSLaunchParent("TX1", [], is_core = True)
parent.start()
print("\n--------------------------------------\n\n")
bridge = CvBridge()


# Receives a frame from the camera, runs image processing and displays output
def receive_frame(data):
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


# Receives status messages from the flight controller and sends them to log file
def receive_status_msg(data):
    f.write(data.msg + "\n")


# Cleanly exit program
def shutdown_program():
    cv2.destroyAllWindows()
    parent.shutdown()
    f.close()
    lock.close()
    os._exit(0)


if __name__== '__main__':
    # Random token to ensure a new instance has been run
    if len(sys.argv) > 1:
        f.write("Random Token: %s\n" % sys.argv[1])

    try:
        # Initialize ROS
        rospy.init_node("img_proc")
        rospy.on_shutdown(shutdown_program)
        sb = rospy.Subscriber('image', Image, receive_frame, queue_size = 10)
        sb2 = rospy.Subscriber('rosout', Log, receive_status_msg, queue_size = 10)
        pb = rospy.Publisher('events', String, queue_size = 10)
        rospy.Rate(10)

        # Receive event commands
        while not rospy.is_shutdown():
            x = raw_input()
            pb.publish(x)
            if (x == "stop"):
                cv2.destroyAllWindows()
            elif (x == "kill"):
                os.kill(os.getpid(), signal.SIGINT)

    except Exception as e:
        print("ERROR: Terminated main.py with exception %s\n")
        shutdown_program()
