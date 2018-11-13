import message_filters
import time
import os 
import sys 
import tty, termios 
import time 
from std_msgs.msg import String
import rospy
from geometry_msgs.msg import Twist,Point32,Quaternion

order = rospy.Publisher('/order_publish',String,queue_size = 1)
vel = rospy.Publisher('/RosAria/cmd_vel',Twist,queue_size = 1)
rospy.init_node('order',anonymous = True)

def order_publisher():
    
    while True: 
        fd=sys.stdin.fileno() 
        old_settings=termios.tcgetattr(fd)
        try: 
            tty.setraw(fd) 
            ch=sys.stdin.read(1) 
        finally: 
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if (ch=='I' or ch == 'i'): 
            print 'move forward'   
        elif (ch=='q' or ch == 'Q'): 
            print "shutdown!" 
            #break 
        elif (ord(ch)==0x3): 
            break 
        else:
            print 'stop'
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x =0
            twist.angular.y =0 
            twist.angular.z =0
            vel.publish(twist)
        order.publish(ch)


if __name__ == '__main__':
    try:
        order_publisher()
    except rospy.ROSInterruptException:
        pass




