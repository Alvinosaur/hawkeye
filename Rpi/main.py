import time
import rospy
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import threading
import signal
import os
import logging
import sys
import zc.lockfile


lock = zc.lockfile.LockFile('lock')
start_event = threading.Event()
stop_event = threading.Event()
f = open("capstone_rpi_log.txt", "w", buffering=0)


def handle_event(s):
    f.write("Received event %s\n" % s)
    if (s.data == "start"):
        f.write("Processing start event...\n")
        start_event.set() 
    elif (s.data == "stop"):
        f.write("Processing stop event...\n")
        stop_event.set()
    #elif (s.data == "kill"):
    #    f.write("Processing kill event...\n")
    #    os.kill(os.getpid(), signal.SIGINT)


def startup_camera():
    f.write("Starting up camera!\n")
    width = 426
    height = 240 
    camera = PiCamera()
    camera.resolution = (width, height)
    camera.framerate = 30
    camera.image_effect = "saturation"
    # DO NOT REMOVE THIS- GRACEFULLY SHUT DOWN CAMERA
    rospy.on_shutdown(lambda: shutdown_ros(camera))
    time.sleep(1) # sleep for 2 seconds to initialize camera hardware
    return camera


def shutdown_camera(camera):
    f.write("Stopping camera!\n")
    camera.close()


def shutdown_ros(camerai = None):
    if camera is not None:
        shutdown_camera(camera)
    f.write("Shutting down ROS!\n")
    f.close()
    lock.close()
    os._exit(0)


if __name__== '__main__':
    signal.signal(signal.SIGTSTP, shutdown_ros)

    # Random token to ensure a new instance has been run
    if len(sys.argv) > 1:
        f.write("Random Token: %s\n" % sys.argv[1])

    # Initialize ROS
    f.write("Initializing ROS!\n")
    # ipdb.set_trace()
    rospy.init_node("img_conv")
    image_pub = rospy.Publisher('image',Image, queue_size=10) 
    sb = rospy.Subscriber('events', String, handle_event, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        start_event.wait()
        start_event.clear()
        camera = startup_camera()
        try: 
            while not rospy.is_shutdown():
                if stop_event.isSet():
                    stop_event.clear()
                    shutdown_camera(camera)
                    break
                cur_time = time.time()
                # 2 second delay between grabbing images
                # rate.sleep()
                # grab image
                raw_capture = PiRGBArray(camera)
                camera.capture(raw_capture, 'rgb')
                image_pub.publish(bridge.cv2_to_imgmsg(raw_capture.array, "rgb8"))

        except:
            shutdown_camera(camera)


