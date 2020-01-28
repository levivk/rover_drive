#!/usr/bin/env python

import rospy
from rover_drive.msg import drive_vel
from sensor_msgs.msg import Joy
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from fibre.utils import Event, Logger


SPEED_LIMIT = 1000

class Driver():

    def __init__(self, timeout):
        
        # # Get odrives
        # rospy.loginfo("Looking for ODrives...")

        self.SERIAL_NUMS = [
            12345,                  # Left, 0
            35623406809166,         # Middle, 1
            34567]                  # Right, 2

        self.odrvs = [
            None,
            None,
            None]

        # Get ODrives
        done_signal = Event(None)

        def discovered_odrv(obj):
            print("Found odrive with sn: {}".format(obj.serial_number))
            if obj.serial_number in self.SERIAL_NUMS:
                self.odrvs[self.SERIAL_NUMS.index(obj.serial_number)] = obj
                print("ODrive is # {}".format(self.SERIAL_NUMS.index(obj.serial_number)))
            else:
                print("ODrive sn not found in list. New ODrive?")
            if not None in self.odrvs:
                done_signal.set()

        odrive.find_all("usb", None, discovered_odrv, done_signal, None, Logger(verbose=False))
        # Wait for ODrives
        try:
            done_signal.wait(timeout=30)
        finally:
            done_signal.set()

        # self.odrv0 = odrive.find_any()
        # # odrv1 = odrive.find_any()
        # # odrv2 = odrive.find_any()
        rospy.loginfo("Found ODrives")

        # Set left and right axis
        self.leftAxes = [self.odrvs[0].axis0, self.odrvs[0].axis1, self.odrvs[1].axis0]
        self.rightAxes = [self.odrvs[1].axis1, self.odrvs[2].axis0, self.odrvs[2].axis1]
        self.axes = self.leftAxes + self.rightAxes

        # Set axis state
        rospy.logdebug("Setting velocity control")
        for ax in (self.leftAxes + self.rightAxes):
            ax.watchdog_feed()

        # Clear errors
        for odrv in self.odrvs:
            dump_errors(odrv, True)

        for ax in (self.leftAxes + self.rightAxes):
            ax.controller.vel_ramp_enable = True
            ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            ax.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

        # Sub to topic
        rospy.Subscriber('joy', Joy, self.vel_callback)

        # Set first watchdog
        self.timeout = timeout  # log error if this many seconds occur between received messages
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)
        self.watchdog_fired = False

        # Init other variables
        self.filter_old_msgs = False
        self.time_offset = 0

        rospy.loginfo("Ready for topic")
        rospy.spin()

    def vel_callback(self, data):

        msg_time = data.header.stamp.to_sec()

        if (self.watchdog_fired == True):
            self.watchdog_fired = False
            self.conn_lost_dur = rospy.Time.now() - self.conn_lost_time
            rospy.logwarn("Connection to controller reestablished! Lost connection for {} seconds.".format(self.conn_lost_dur.to_sec()))
            self.filter_old_msgs = True

        if (self.filter_old_msgs):
            # if old messages are being sent due to connection loss
            if (msg_time < (rospy.Time.now().to_sec() - self.time_offset - 0.25)):
                # And if the time difference is significantly greater than the average offset
                # Ignore this msg
                rospy.logwarn("Ignoring {} second old message".format((rospy.Time.now().to_sec() - self.time_offset) - msg_time))
                return
            else:
                # Msg is current, stop ignoring
                self.filter_old_msgs = False

        else:
            # Get time from msg and find offset
            offset = rospy.Time.now().to_sec() - msg_time
            self.time_offset = (self.time_offset * 0.8) + (offset * 0.2) # Create average time offset
            rospy.logdebug("Average control time offset: {}".format(self.time_offset))


        # Do stuff for all axes
        for ax in self.axes:
            ax.watchdog_feed()

            # # ODrive watchdog error clear
            # if(ax.error == errors.axis.ERROR_WATCHDOG_TIMER_EXPIRED):
            #     ax.error = errors.axis.ERROR_NONE
            #     rospy.logwarn("Cleared ODrive watchdog error")
            # For other errors
            if (ax.error != errors.axis.ERROR_NONE):
                rospy.logfatal("Received axis error: {} {}".format(self.axes.index(ax), ax.error))

        # Emergency brake - 4 & 5 are bumpers
        if (data.buttons[4] and data.buttons[5]):
            # Stop motors
            rospy.logdebug("Applying E-brake")
            for ax in (self.leftAxes + self.rightAxes):
                ax.controller.vel_ramp_target = 0
                ax.controller.vel_setpoint = 0

        # Control motors as tank drive
        for ax in self.leftAxes:
            ax.controller.vel_ramp_target = data.axes[1] * SPEED_LIMIT
            ax.watchdog_feed()
        for ax in self.rightAxes:
            ax.controller.vel_ramp_target = data.axes[4] * SPEED_LIMIT
            ax.watchdog_feed()

        rospy.loginfo(rospy.get_caller_id() + "Left: %s", data.axes[1] * 1000)
        rospy.loginfo(rospy.get_caller_id() + "Right: %s", data.axes[4] * 1000)

        # Received mesg so reset watchdog
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)

    def watchdog_callback(self, event):
        # Have not received mesg for self.timeout seconds
        self.conn_lost_time = rospy.Time.now()
        rospy.logwarn("Control timeout! {} seconds since last control!".format(self.timeout))
        self.watchdog_fired = True

        # Stop motors
        for ax in self.leftAxes:
            ax.controller.vel_ramp_target = 0
            ax.controller.vel_setpoint = 0
        for ax in self.rightAxes:
            ax.controller.vel_ramp_target = 0
            ax.controller.vel_setpoint = 0

    def clear_errors(self, odrv):
        dump_errors(odrv, True)


if __name__ == '__main__':
    rospy.init_node('driver')
    timeout = 2
    driver = Driver(timeout)
