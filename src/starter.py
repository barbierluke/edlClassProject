#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
# Simply pings the motor_controller node to check the status of the tiva


class Starter:
    def __init__(self):
        rospy.init_node('starter_node')
        pub = rospy.Publisher('/start', Empty, queue_size=10)
        rospy.sleep(3)
        pub.publish(Empty())
        rospy.loginfo("Message Published")
        # Do nothing here after we start everything...
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == "__main__":
    s = Starter()
