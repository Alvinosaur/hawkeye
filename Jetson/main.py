from roslaunch.parent import ROSLaunchParent
import RPi.GPIO as GPIO
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
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

# Initialization
lock = zc.lockfile.LockFile('lock')     # Ensures two instances of program can't be run simultaneously
f = open("/home/siddesh/hawkeye/Jetson/px4_log.txt", "w", buffering=0)

# Launch Roscore
parent = ROSLaunchParent("TX1", [], is_core = True)
parent.start()
print("\n--------------------------------------\n")
bridge = CvBridge()

#Pin definitions
but2_pin = 15
but_pin = 18


# GPIO interrupt handlers
def launch_kill(pb, channel):
    val = GPIO.input(but2_pin)
    if (val == 1):
        send_event(pb, "launch")
    if (val == 0):
        shutdown_program()
def start_stop(pb, channel):
    val = GPIO.input(but_pin)
    if (val == 1):
        send_event(pb, "cam_start")
    if (val == 0):
        send_event(pb, "cam_stop")


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


# Send commnd to Raspberry Pi
def send_event(pb, event):
    print("Sending event %s" % event)
    pb.publish(event)


# Receives acknowlgement from Raspberry Pi
def receive_ack(s):
    if (s.data == "ACK"):
        print("\nReceived acknowledgement from Rpi")
        sys.stdout.write(">> ")
    else:
        print("\nRpi terminated with error")
        print("Error message: %s" % s.data)
        sys.stdout.write(">> ")
    sys.stdout.flush()

# Set flight mode
def set_flight_mode(service, mode):
    service(custom_mode=mode)


# Cleanly exit program
def shutdown_program():
    print("\nKilling program\n")
    cv2.destroyAllWindows()
    parent.shutdown()
    GPIO.cleanup()
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
        sb3 = rospy.Subscriber('ack', String, receive_ack, queue_size=10)
        pb = rospy.Publisher('events', String, queue_size = 10)
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.Rate(10)

        # Initialize GPIO for button input
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(but2_pin, GPIO.IN)
        GPIO.setup(but_pin, GPIO.IN)
        GPIO.add_event_detect(but_pin, GPIO.FALLING, callback=lambda x: start_stop(pb, x), bouncetime=50)
        GPIO.add_event_detect(but2_pin, GPIO.BOTH, callback=lambda x: launch_kill(pb, x), bouncetime=50)

        # Receive event commands via console just in case buttons don't work
        print("Possible commands:")
        print("\tlaunch \t\t-> launch communication b/w Rpi and drone")
        print("\tcam_start \t-> start camera (launch must run first)")
        print("\tcam_stop \t-> stop camera if active")
        print("\tkill \t\t-> kill TX1 process\n")
    except Exception as e:
        print("ERROR: Terminated main.py with exception %s" % e)

    while not rospy.is_shutdown():
        try:    
            x = raw_input(">> ")
            send_event(pb, x)
            if (x == "stop"):
                cv2.destroyAllWindows()
            elif (x == "kill"):
                os.kill(os.getpid(), signal.SIGINT)
            elif (x == "land"):
                set_flight_mode(flightModeService, State.MODE_PX4_LAND)
            elif (x == "return"):
                set_flight_mode(flightModeService, State.MODE_PX4_RTL)
            elif (x == "hold"):
                set_flight_mode(flightModeService, State.MODE_PX4_LOITER)

        except Exception as e:
            print("ERROR: Received exception %s" % e)
