#!/usr/bin/env python

import rospy
from rover_drive.msg import drive_vel

def controller():
    pub = rospy.Publisher('drive_velocity', drive_vel, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10)
    count = 0
    while not rospy.is_shutdown():
        left = count
        right = 0 - count
        data = drive_vel()
        data.left = left
        data.right = right
        pub.publish(data)
        rospy.loginfo(data)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
