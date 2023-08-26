import rospy
from sensor_msgs.msg import Image, Imu
import cv2
from cv_bridge import CvBridge
import argparse

class Stereo2Mono:
    def __init__(self, new_cam_name, new_imu_name):
        self.bridge = CvBridge()

        cam_subscriber_topic = "/cam0/image_raw"
        cam_publisher_topic = new_cam_name

        imu_subscriber_topic = "/imu0"
        imu_publisher_topic = new_imu_name

        #self.cam_subscriber = rospy.Subscriber(cam_subscriber_topic, Image, self.cam_cb, queue_size=1, buff_size=2**24)
        #self.cam_publisher = rospy.Publisher(cam_publisher_topic, Image, queue_size=1)

        self.imu_subscriber = rospy.Subscriber(imu_subscriber_topic, Imu, self.imu_cb, queue_size=1, buff_size=2**24)
        self.imu_publisher = rospy.Publisher(imu_publisher_topic, Imu, queue_size=1)

    def cam_cb(self, data):
        img = self.bridge.imgmsg_to_cv2(data)
        img = cv2.resize(img, (324, 244), interpolation=cv2.INTER_AREA)

        new_data = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        new_data.header = data.header
        #new_data.height = data.height
        #new_data.width = data.width
        self.cam_publisher.publish(new_data)

    def imu_cb(self, data):
        #print(data)
        self.imu_publisher.publish(data)

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--camera-name', default="/camera/image_raw", type=str)
    parser.add_argument('-i', '--imu-name', default="/imu", type=str)
    args = parser.parse_args()

    cam_name = args.camera_name
    imu_name = args.imu_name

    rospy.init_node("stereo2mono_node", anonymous=True)
    s2m = Stereo2Mono(cam_name, imu_name)
    rospy.sleep(0.1)
    rospy.spin()
