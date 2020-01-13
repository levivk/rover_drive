#!/usr/bin/env python

import rospy
from rover_drive.msg import drive_vel

def vel_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "Left: %s", data.left)
    rospy.loginfo(rospy.get_caller_id() + "Right: %s", data.right)

def driver():
    rospy.init_node('driver')
    rospy.Subscriber('drive_velocity', drive_vel, vel_callback)
    rospy.loginfo("About to spin")
    rospy.spin()

if __name__ == '__main__':
    rospy.loginfo("Starting...")
    driver()