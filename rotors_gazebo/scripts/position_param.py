#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose

def callback(data):
    rospy.set_param('/bebop2/odom_param/x', data.position.x)
    rospy.set_param('/bebop2/odom_param/y', data.position.y)
    rospy.set_param('/bebop2/odom_param/z', data.position.z)

    
def listener():
    rospy.init_node('position_param', anonymous=True)

    rospy.Subscriber("/bebop2/ground_truth/pose", Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()