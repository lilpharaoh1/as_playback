import numpy as np
import os
from glob import glob
import rospy
from sensor_msgs.msg import Image
import argparse
import cv2
from cv_bridge import CvBridge

class ASVideoPlayback:
    TIMER_DURATION = 0.5 # Check on AirSim
    IMAGE_PATH = "/home/lilpharaoh1/as_ws/data/"

    def __init__(self, folder, grey=True):
        self.images = []
        self.image_idx = 0
        self.full_path = self.IMAGE_PATH + folder + "/grey" if grey else self.IMAGE_PATH + folder
        self.bridge = CvBridge()

        for filename in sorted(os.listdir(self.full_path)):
            print(self.full_path + "/" + filename)
            image = cv2.imread(self.full_path + "/" + filename)
            #image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            self.images.append(image)
    
        image_topic = "/camera/image_raw"
        image_msg_type = Image

        self.image_pub = rospy.Publisher(image_topic, image_msg_type, queue_size=1)
        self.playback_timer = rospy.Timer(rospy.Duration(self.TIMER_DURATION), self.image_cb)

    def image_cb(self, _):
        try:
            image = self.images[self.image_idx]
        except:
            print("reseting self.image_idx...")
            self.image_idx = 0
            image = self.images[self.image_idx]
        print(image.shape)
        msg = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
        msg.header.stamp = rospy.Time().now()
        self.image_pub.publish(msg)
        
        self.image_idx += 1

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--folder', default="NO-FOLDER-GIVEN", type=str)
    parser.add_argument('-g', '--grey', default=True, type=bool)
    args = parser.parse_args()

    folder = args.folder
    if folder == "NO-FOLDER-GIVEN":
        raise Exception("No folder given, please use -f [FOLDER NAME]")
    grey = args.grey

    rospy.init_node("video_playback_node", anonymous=True)
    disparity = ASVideoPlayback(folder, grey=grey)
    rospy.sleep(0.1)
    rospy.spin()
