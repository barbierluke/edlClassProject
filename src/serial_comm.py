import serial
import rospy
from dream_team.msg import Wheel
from std_msgs.msg import UInt8
from std_msgs.msg import Empty

"""
Listens to the Tiva's state and publishes it to /state as a uint8
Subscribes to /wheel and decodes the messages to send to the tiva
"""

class Serial_Comm:

    def __init__(self, ttyStr='/dev/tiva'):
        rospy.init_node('motor_controller')
        self.sub = rospy.Subscriber('/wheel', Wheel, self.wheel_callback)
        self.sub = rospy.Subscriber('/ping', Empty, self.ping_callback)
        self.pub = rospy.Publisher('/state', UInt8, queue_size=10)
        self.tiva = serial.Serial(ttyStr,timeout=0.1) # Should do 9600,8,N,1

        self.prev_state = -1
        rospy.spin()

    def ping_callback(self,msg):
        state = int(self.tiva.read(1))
        rospy.loginfo("State: "+str(state))

        # The state has changed so publish it
        if state != self.prev_state:
            msg = UInt8()
            msg.data = state
            self.pub.publish(msg)
        self.prev_state = state
        
    def wheel_callback(self, msg):
        left = msg.left
        right = msg.right
        c = encode_msg(left, right)
        if c != None:
            self.tiva.write(c)

    def encode_msg(left, right):
        if left > 5 or right > 5:
            rospy.logerr("Wheel speed higher than possible: " + str(left) + " " + str(right))
            return None
        if left==0 and right==0:
            return 'a'
        sum = left+right
        c = (sum - 5)*2 + 98
        if right > left:
            c = c + 1
        return chr(c)

if __name__ == "__main__":
    a = Serial_Comm()
