import time
import rospy
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
# import ipdb


if __name__== '__main__':
    # ipdb.set_trace()
    width = 426
    height = 240
    camera = PiCamera()
    # DO NOT REMOVE THIS- GRACEFULLY SHUT DOWN CAMERA
    rospy.on_shutdown(camera.close)
    #camera.awb_mode = 'off'
   # camera.awb_gains = (0.5, 0.5)
    camera.resolution = (width, height)
    camera.framerate = 30
    rospy.init_node("img_conv")
    image_pub = rospy.Publisher('image', CompressedImage, queue_size=10) 
    rate = rospy.Rate(30)
    time.sleep(1) # sleep for 2 seconds to initialize camera hardware
    cur_time = time.time()

    saved_prev = False

    while not rospy.is_shutdown():
        cur_time = time.time()
        # 2 second delay between grabbing images
        # rate.sleep()
        # grab image
        raw_capture = PiRGBArray(camera)
        camera.capture(raw_capture, 'bgr')
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', raw_capture.array)[1]).tostring()
        image_pub.publish(msg)




