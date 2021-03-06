#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from math import exp, log1p, ceil
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Empty
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from pdb import set_trace

    

class Image_Processing:
    def __init__(self):
        rospy.init_node("line_follower", anonymous = True)
        rospy.Subscriber("/image_raw", Image, self.ros_to_cv)
        rospy.Subscriber("/cv", Empty, self.respond_current_speed)
        self.image_pub = rospy.Publisher('/cv_move_left', Float32, queue_size=20)
        self.bridge = CvBridge()
        rospy.loginfo("Line Follower: Starting CV Analysis")
        rospy.spin()

    def respond_current_speed(self, msg):
        rospy.loginfo("Line Follower: Giving CV msg")
        msg = Float32()
        msg.data = self.wilbur
        self.image_pub.publish(msg)
        
    def ros_to_cv(self, ros_image):
#        rospy.loginfo("line follower: image received")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        image_binary = self.make_binary(cv_image)
        self.wilbur = self.get_data(image_binary)
        rospy.sleep(0.1)
        

    def make_binary(self,image):
        img_greyscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        retval, img_binary = cv2.threshold(image, 50, 80, cv2.THRESH_BINARY)
        #cv2.imshow("Threshold Binary", img_binary)
        #img_binary = cv2.cvtColor(img_binary, cv2.COLOR_BGR2GRAY)
        
        #cv2.imshow("Greyscale #GOT", img_greyscale)
        #cv2.imshow("Binary Image", img_binary)
        #cv2.waitKey(5)

        return img_binary

    def get_data(self,image):

        (rows, cols, channels) = image.shape
        column_sums=np.sum(image, axis = 0)
        min_index = np.nanargmin(column_sums)

        wilbur = np.around((.97/1920)*min_index +.01, 2)
        return wilbur
        
                
        

if __name__ == '__main__':
    a = Image_Processing()
    #cv2.destroyAllWindows()
