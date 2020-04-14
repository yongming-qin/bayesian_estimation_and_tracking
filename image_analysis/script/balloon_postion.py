#!/home/yq/anaconda3/envs/rcnn/bin/python
"""
/home/yq/anaconda3/envs/rcnn/bin/python
First do instance segmentation of the balloons using mask rccn.
Then map the pixel positions in the point cloud data.
Yongming Qin
2020/01/11
"""

import os
import sys
import rospy
import math


from sensor_msgs.msg import Image
import numpy as np
import cv2
import skimage.draw
# convert sensor_msgs.msg.Image to cv image
from cv_bridge import CvBridge, CvBridgeError 
bridge = CvBridge()

file_dirname = os.path.dirname(os.path.abspath(__file__))
mask_rcnn_path = os.path.join(file_dirname, "../Mask_RCNN/samples/balloon/")
sys.path.append(mask_rcnn_path)
import yq_balloon
instance_segmentation = yq_balloon.InstanceSegmentation()

image = skimage.io.imread("/home/yq/Documents/ZED/first_record/left000001.png")
# print(image.shape)
instance_segmentation.process(image)

cnt = 0
def callback(data):
    global cnt
    cnt = cnt + 1
    if cnt % 100 == 0:
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        ret = instance_segmentation.process(cv_image)
        

if __name__ == "__main__":
    rospy.init_node("balloons_segmentation")
    rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, callback)
    rospy.spin()
    

    