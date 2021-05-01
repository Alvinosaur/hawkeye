import rospy
from sensor_msgs.msg import Image

prev_time = None
diffs = []

def get_image(data):
    global prev_time
    cur_time = data.header.stamp.to_sec()
    if prev_time is not None:
        diff = cur_time - prev_time
        diffs.append(diff)
    prev_time = cur_time

rospy.init_node("get_fps")
sb = rospy.Subscriber('image', Image, get_image, queue_size=10)
while True:
    x = raw_input()
    print(len(diffs) / sum(diffs))


