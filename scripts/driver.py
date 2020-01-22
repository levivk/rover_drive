#!/usr/bin/env python

import rospy
from rover_drive.msg import drive_vel
from sensor_msgs.msg import Joy
import odrive
from odrive.enums import *

odrv0 = None

class Driver():

    def __init__(self, timeout):
        
        # Get odrives
        rospy.loginfo("Looking for ODrives...")
        self.odrv0 = odrive.find_any()
        # odrv1 = odrive.find_any()
        # odrv2 = odrive.find_any()
        rospy.loginfo("Found ODrives")

        # Set left and right axis
        self.leftAxes = [odrv0.axis0]
        self.rightAxes = [odrv0.axis1]

        # # Set axis state
        # rospy.loginfo("Setting velocity control")
        # for ax in leftAxes:
        #     ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     ax.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        # for ax in rightAxes:
        #     ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #     ax.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        # Sub to topic
        rospy.Subscriber('joy', Joy, self.vel_callback)

        # Set first watchdog
        self.timeout = timeout  # log error if this many seconds occur between received messages
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)
        self.watchdog_fired = False

        rospy.loginfo("Ready for topic")
        rospy.spin()

    def vel_callback(self, data):
        if (self.watchdog_fired == True):
            self.watchdog_fired = False
            self.conn_lost_dur = rospy.Time.now() - self.conn_lost_time
            rospy.logwarn("Connection to controller reestablished! Lost connection for {} seconds.".format(self.conn_lost_dur.to_sec()))

        for ax in self.leftAxes:
            ax.controller.vel_setpoint = data.axes[1] * 1000
            ax.watchdog_feed()
        for ax in self.rightAxes:
            ax.controller.vel_setpoint = data.axes[4] * 1000
            ax.watchdog_feed()

        rospy.loginfo(rospy.get_caller_id() + "Left: %s", data.axes[1] * 1000)
        rospy.loginfo(rospy.get_caller_id() + "Right: %s", data.axes[4] * 1000)

        # Received mesg so reset watchdog
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)

    def watchdog_callback(self, event):
        # Have not received mesg for self.timeout seconds
        rospy.logwarn("Control timeout! {} seconds since last control!".format(self.timeout))
        self.watchdog_fired = True
        self.conn_lost_time = rospy.Time.now()


if __name__ == '__main__':
    rospy.init_node('driver')
    timeout = 2
    driver = Driver(timeout)