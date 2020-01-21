#!/usr/bin/env python

import rospy
from rover_drive.msg import drive_vel
import odrive
from odrive.enums import *

leftAxes = []
rightAxes = []
odrv0 = None

def vel_callback(data):
    for ax in leftAxes:
        rospy.loginfo('changing left')
        ax.controller.vel_setpoint = data.axes[1]
    for ax in rightAxes:
        rospy.loginfo('changing right')
        ax.controller.vel_setpoint = data.axes[4]
    
    rospy.loginfo(rospy.get_caller_id() + "Left: %s", data.left)
    rospy.loginfo(rospy.get_caller_id() + "Right: %s", data.right)

def driver():
    rospy.init_node('driver')

    # Get odrives
    rospy.loginfo("Looking for ODrives...")
    odrv0 = odrive.find_any()
    # odrv1 = odrive.find_any()
    # odrv2 = odrive.find_any()
    rospy.loginfo("Found ODrives")

    # Set left and right axis
    leftAxes.extend([odrv0.axis0])
    rightAxes.extend([odrv0.axis1]) 

    # # Set axis state
    # rospy.loginfo("Setting velocity control")
    # for ax in leftAxes:
    #     ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #     ax.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    # for ax in rightAxes:
    #     ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    #     ax.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    # Sub to topic
    rospy.Subscriber('control_data', Joy, vel_callback)

    rospy.loginfo("Ready for topic")
    rospy.spin()

if __name__ == '__main__':
    driver()