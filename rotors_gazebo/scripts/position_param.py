#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

def pose_callback(data):
    rospy.set_param('/bebop2/odom_param/position/x', data.position.x)
    rospy.set_param('/bebop2/odom_param/position/y', data.position.y)
    rospy.set_param('/bebop2/odom_param/position/z', data.position.z)
    rospy.set_param('/bebop2/odom_param/orientation/w', data.orientation.w)
    rospy.set_param('/bebop2/odom_param/orientation/x', data.orientation.x)
    rospy.set_param('/bebop2/odom_param/orientation/y', data.orientation.y)
    rospy.set_param('/bebop2/odom_param/orientation/z', data.orientation.z)


def imu_callback(data):
    rospy.set_param('/bebop2/odom_param/linear/x', data.twist.twist.linear.x)
    rospy.set_param('/bebop2/odom_param/linear/y', data.twist.twist.linear.y)
    rospy.set_param('/bebop2/odom_param/linear/z', data.twist.twist.linear.z)
    rospy.set_param('/bebop2/odom_param/angular/x', data.twist.twist.angular.x)
    rospy.set_param('/bebop2/odom_param/angular/y', data.twist.twist.angular.y)
    rospy.set_param('/bebop2/odom_param/angular/z', data.twist.twist.angular.z)


    
def listener():
    rospy.init_node('position_param', anonymous=True)

    rospy.Subscriber("/bebop2/ground_truth/pose", Pose, pose_callback)
    rospy.Subscriber("/bebop2/ground_truth/Odometry", Imu, imu_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()