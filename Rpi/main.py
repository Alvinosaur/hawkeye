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
import sys
import zc.lockfile
import subprocess


# Initialization
lock = zc.lockfile.LockFile('lock')     # Ensures two instances of program can't be run simultaneously
px4_launch_event = threading.Event()    
cam_start_event = threading.Event()
cam_stop_event = threading.Event()
f = open("/home/pi/hawkeye/Rpi/rpi_log.txt", "w", buffering=0)
camera = None
px4_process = None


def handle_event(s):
    f.write("Received event %s\n" % s)
    if (s.data == "launch"):
        f.write("Processing PX4 launch event...\n")
        px4_launch_event.set()
    elif (s.data == "cam_start"):
        f.write("Processing camera start event...\n")
        cam_start_event.set() 
    elif (s.data == "cam_stop"):
        f.write("Processing camera stop event...\n")
        cam_stop_event.set()


def clear_all_events():
    px4_launch_event.clear()
    cam_start_event.clear()
    cam_stop_event.clear()


# Launch communications between Rpi and flight controller
def launch_px4():
    global px4_process
    f.write("Launching communications with Pixhawk!\n")
    px4_process = subprocess.Popen("/home/pi/hawkeye/Rpi/px4_launch.sh")


# Start the camera so that we can begin to stream video
def startup_camera():
    global camera
    f.write("Starting up camera!\n")
    width = 426
    height = 240 
    camera = PiCamera()
    camera.resolution = (width, height)
    camera.framerate = 30
    camera.image_effect = "saturation" 
    time.sleep(1) # sleep to initialize camera hardware


# Cleanly close the camera once streaming is complete
def shutdown_camera():
    global camera
    f.write("Stopping camera!\n")
    if camera is not None:
        camera.close()
        camera = None


# Cleanly exit program
def shutdown_program():
    f.write("Shutting down program!\n")
    if camera is not None:
        shutdown_camera()
    if px4_process is not None:
        f.write("Terminating communications with pixhawk!\n")
        px4_process.terminate() 
    f.close()
    lock.close()
    os._exit(0)


if __name__== '__main__':
    signal.signal(signal.SIGTSTP, lambda x, y: shutdown_program())

    # Random token to ensure a new instance has been run
    if len(sys.argv) > 1:
        f.write("Random Token: %s\n" % sys.argv[1])

    try:
        # Initialize ROS
        f.write("Initializing ROS!\n")
        rospy.init_node("img_conv")
        image_pub = rospy.Publisher('image',Image, queue_size=10) 
        sb = rospy.Subscriber('events', String, handle_event, queue_size=10)
        bridge = CvBridge()
        rate = rospy.Rate(10)
        rospy.on_shutdown(shutdown_program)

        # Wait for PX4 communications to be launched
        px4_launch_event.wait()
        clear_all_events()
        launch_px4()

        while not rospy.is_shutdown():
            # Wait for camera startup
            cam_start_event.wait()
            clear_all_events()
            startup_camera()

            # Continuously stream video until indicated to shutdown
            while not rospy.is_shutdown():
                if cam_stop_event.isSet():
                    clear_all_events()
                    shutdown_camera()
                    break
                raw_capture = PiRGBArray(camera)
                camera.capture(raw_capture, 'rgb')
                image_pub.publish(bridge.cv2_to_imgmsg(raw_capture.array, "rgb8"))

    except Exception as e:
        error_msg = "ERROR: Terminated main.py with exception %s\n" % str(e)
        f.write(error_msg)
        shutdown_program()


