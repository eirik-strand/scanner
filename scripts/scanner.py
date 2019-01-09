#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

import cv2
import numpy as np

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import pcl


def image_CB():
    print("Katt")


'''
def img_to_cloud(image, coords):

    cloud = pcl.PointCloud_PointXYZRGB()



    h = image.shape[0]
    w = image.shape[1]
    
    for y in range(0, h):
        for x in range(0, w):

            point = pcl.Point() 
            color = cv2.Vec3b()

            color = image[y,x ]

            r = (color[2]);
            g = (color[1]);
            b = (color[0]);


            point.x = x
            point.y = y;
            point.z = 1;


            
            cloud.points.push_back(point);
            
    return cloud;


'''

if __name__ == '__main__':
    rospy.init_node('scanner')
    '''
    try:
        image_CB()
    except rospy.ROSInterruptException:
        pass
    '''
    rospy.spin()
    # image_CB()