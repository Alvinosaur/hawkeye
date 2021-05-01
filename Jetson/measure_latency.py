import rospy
from sensor_msgs.msg import Image

prev_time = None
diffs = []

def get_image(data):
    send_time = data.header.stamp.to_sec()
    cur_time = rospy.Time.now().to_sec()
    diff = cur_time - send_time
    diffs.append(diff)

rospy.init_node("get_fps")
sb = rospy.Subscriber('image', Image, get_image, queue_size=10)
while True:
    x = raw_input()
    print(sum(diffs) / len(diffs))


