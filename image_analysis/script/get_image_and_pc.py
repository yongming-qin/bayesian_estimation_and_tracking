#!/home/yq/anaconda3/envs/rcnn/bin/python
"""
Using message_filter package to get one meassage from image and pc (point cloud) topic
at the same time. I then process image and find the corresponding part from
the pc.

Yongming Qin
2020/01/12
"""
import os
import sys
print(sys.version)
import rospy
import numpy as np
import ros_numpy


import message_filters
from sensor_msgs.msg import Image, PointCloud2
import cv2
import skimage.draw

# convert sensor_msgs.msg.Image to cv image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

# mask rcnn
file_dirname = os.path.dirname(os.path.abspath(__file__))
mask_rcnn_path = os.path.join(file_dirname, "../Mask_RCNN/samples/balloon/")
sys.path.append(mask_rcnn_path)
import yq_balloon
instance_segmentation = yq_balloon.InstanceSegmentation()
image = skimage.io.imread("/home/yq/Documents/ZED/first_record/left000001.png")
instance_segmentation.process(image)
print("-----------------------------Start Subscribing--------------------------------------------")

cnt = 0
HEIGHT = 376
WIDTH = 672
xyz_array = np.empty( (HEIGHT, WIDTH, 3) )
def callback(image, pc):
    global cnt
    cnt = cnt + 1
    if cnt % 100 == 1:
        print(image.height, image.width)
        try:
            cv_image = bridge.imgmsg_to_cv2(image, "bgr8")
            print(cv_image.shape)
        except CvBridgeError as e:
            print(e)
        
        ret = instance_segmentation.process(cv_image)
        #print(pc.height, pc.width, type(pc.data))

        array_tuple = ros_numpy.numpify(pc)
        global xyz_array
        for i in range(HEIGHT):
            for j in range(WIDTH):
                for k in range(3):
                    xyz_array[i,j,k] = array_tuple[i,j][k]
        nans = np.full_like(xyz_array, np.nan, dtype=np.double)
        
        if ret['masks'].shape[-1] > 0:
            mask = ret['masks'][:,:,0] # shape: (720,1080) / (480, 800)
            print(mask.shape)
            extend_mask = np.repeat(mask, 3).reshape(HEIGHT,WIDTH,3)
            region = np.where(extend_mask, xyz_array, nans)
            position = np.nanmean(region, axis=(0,1))
            if position.any() == np.nan:
                print("position nan")
            else:
                instance_segmentation.splash_and_save(cv_image)
                print(position)
        else:
            print("no mask")




rospy.init_node("get_image_and_pc")
image_sub = message_filters.Subscriber("/zed/zed_node/rgb/image_rect_color", Image)
pc_sub = message_filters.Subscriber("/zed/zed_node/point_cloud/cloud_registered", PointCloud2)
#QIN buffsize affect the results! There may be no matched messages with small buffsize
ts = message_filters.ApproximateTimeSynchronizer([image_sub, pc_sub], 10, 0.01)
ts.registerCallback(callback)

rospy.spin()