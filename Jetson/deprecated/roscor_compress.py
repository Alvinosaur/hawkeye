from roslaunch.parent import ROSLaunchParent
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np

#parent = ROSLaunchParent("TX1", [], is_core = True)
#parent.start()

bridge = CvBridge()
cv2.startWindowThread()
cv2.namedWindow("Tracking")

# Function which takes in an image and runs our processing
def process(data):
    # Conversion Code adapted from tutorial on ROS website
    np_arr = np.fromstring(data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #feat_det = cv2.FastFeatureDetector_create()
    #featPoints = feat_det.detect(cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
    #for featpoint in featPoints:
    #    x, y = featpoint.pt
    #    cv2.circle(image_np, (int(x), int(y)), 3, (0, 0, 255), -1)
    cv2.imshow("Tracking", image_np)

rospy.init_node("img_proc")
rospy.on_shutdown(cv2.destroyAllWindows)
sb = rospy.Subscriber('image', CompressedImage, process, queue_size = 10)
rospy.spin()
