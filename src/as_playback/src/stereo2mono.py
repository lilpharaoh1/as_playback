import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import argparse

class Stereo2Mono:
    def __init__(self, new_topic_name):
        self.bridge = CvBridge()

        subscriber_topic = "/cam0/image_raw"
        publisher_topic = new_topic_name

        self.cam_subscriber = rospy.Subscriber(subscriber_topic, Image, self.sub_cb, queue_size=1, buff_size=2**24)
        self.cam_publisher = rospy.Publisher(publisher_topic, Image, queue_size=1)

    def sub_cb(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        img = cv2.resize(img, (324, 244), interpolation=cv2.INTER_AREA)
        
        new_data = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        new_data.header = data.header
        #new_data.height = data.height
        #new_data.width = data.width
        self.cam_publisher.publish(new_data)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--topic-name', default="/camera/image_raw", type=str)
    args = parser.parse_args()

    topic_name = args.topic_name
    
    rospy.init_node("stereo2mono_node", anonymous=True)
    s2m = Stereo2Mono(topic_name)
    rospy.sleep(0.1)
    rospy.spin()
