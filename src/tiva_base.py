import rospy
import serial
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from std_msgs.msg import Int8
from dream_team.msg import MoveBase

"""
Tiva communicator node

This node functions through a looping callback
3 callbacks (loop, theta, cv_move)
The loop callback gets activated by the starter node (/start)
Loop callback communicates with the tiva and determines CV or TDOA
it publishes to those nodes and they reply on /tdoa or /cv in respective callbacks
theta and cv_move communicate over serial and call the loop back to start the process over again
"""

class Tiva_Base:
    def __init__(self):
        # initialize serial stuff
        ttyStr='/dev/tiva'
        self.tiva = serial.Serial(ttyStr,timeout=None) # Should do 9600,8,N,1
        self.tiva.reset_input_buffer()
        # initialize ros stuff
        rospy.init_node("tiva_node")
        rospy.Subscriber('/cv_move_left', Float32, self.cv_move_callback)
        rospy.Subscriber('/theta', Int8, self.theta_callback)
        rospy.Subscriber('/start', Empty, self.loop_callback)
        self.tdoa_pub = rospy.Publisher('/tdoa', Int16, queue_size=10)
        self.cv_pub = rospy.Publisher('/cv', Empty, queue_size=10)
        self.loop_pub = rospy.Publisher('/start', Empty, queue_size=10)
        rospy.loginfo("tiva node ready")
        rospy.spin()
            
    def loop_callback(self, ros_msg):
        rospy.loginfo("loop callback")
        msg = self.tiva.read(4) # read in 4 chars
#        print(msg)
        
        if msg[0] is '1': # TDOA
            rospy.loginfo("Tiva wants TDOA")
            time_diff = int(msg[1:4]) # get the tdoa
            pubMsg = Int16()
            pubMsg.data = time_diff
            self.tdoa_pub.publish(pubMsg)
        elif msg[0] is '2': # CV
            rospy.loginfo("Tiva wants CV")
            self.cv_pub.publish(Empty())
        else:
            rospy.logerr("Unrecognized 1st char from tiva: " + msg)
#            self.tiva.reset_input_buffer() # reset buffer to get back on track

    def theta_callback(self, msg):
        rospy.loginfo("Entering theta_callback")
        theta = msg.data # int8 -90 - +90
        
        rospy.loginfo("TDOA wants %s degree turn"%theta)
        c1 = abs(theta // 10)
        c2 = theta % 10

        if theta > 0:
            msg = "+" + str(c1) + str(c2)
        elif theta < 0:
            msg = "-" + str(c1) + str(c2)
        else:
            msg = "+00"
        bytes_written = self.tiva.write(msg)
        if bytes_written != 3:
            rospy.logerr("Bytes Written: %s. Should be 3"%bytes_written)
        # loop back
        self.loop_pub.publish(Empty()) # loop again
                         
    def cv_move_callback(self, msg):
        left_wheel = msg.data # this is a float for left wheel
        left_wheel = abs(int(left_wheel * 100))
        c1 = left_wheel // 10
        c2 = left_wheel % 10
        msg = str(c1) + str(c2) + 'x' # first two bits are important, last is dc
        bytes_written = self.tiva.write(msg)
        if bytes_written != 3:
            rospy.logerr("Bytes Written: %s. Should be 3"%bytes_written)
        # loop back
        self.loop_pub.publish(Empty()) # loop again

if __name__ == "__main__":
    a = Tiva_Base()
