#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


storeVel=Twist()
def velocity_callback(msg):
    global storeVel
    storeVel.linear=msg.linear
    storeVel.angular=msg.angular


if __name__=='__main__':
    rospy.init_node('republish_velocity')

    sub=rospy.Subscriber("cmd_vel_orig",Twist,velocity_callback)
    pub=rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        if storeVel:
            pub.publish(storeVel)
            rate.sleep()
