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
import turtlesim.msg


def image_CB():
    print("Katt")





if __name__ == '__main__':
    rospy.init_node('scanner')
    try:
        image_CB()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
    # image_CB()