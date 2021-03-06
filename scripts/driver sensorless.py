#!/usr/bin/env python

import rospy
#from rover_drive.msg import drive_vel
from sensor_msgs.msg import Joy
import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from fibre.utils import Event, Logger
from fibre.protocol import ChannelBrokenException
import time

SPEED_LIMIT = 2000
MSG_PER_SECOND = 60
WD_FEED_PER_SECOND = 2

class Driver():

    def __init__(self, timeout):
        
        # specify left, middle, and right ODrives
        rospy.loginfo("Looking for ODrives...")

        self.SERIAL_NUMS = [
            35593293288011,                  # Left, 0
            35623406809166,                  # Middle, 1
            35563278839886]                  # Right, 2

        rospy.loginfo("Getting odrives first time")
        self.get_odrives()

        rospy.loginfo("Erasing and restarting odrives")
        
        for odrv in self.odrvs:
            odrv.erase_configuration()
            try:
                odrv.reboot()
            except ChannelBrokenException:
                pass

        rospy.loginfo("Reconnecting to odrives")
        self.get_odrives()
        rospy.loginfo("Reconnected!")

        # Set left and right axis
        self.leftAxes = [self.odrvs[0].axis0, self.odrvs[0].axis1, self.odrvs[1].axis1]
        self.rightAxes = [self.odrvs[1].axis0, self.odrvs[2].axis0, self.odrvs[2].axis1]
        self.axes = self.leftAxes + self.rightAxes

        # Set axis state
        for ax in (self.leftAxes + self.rightAxes):
            ax.watchdog_feed()

        # Clear errors
        for odrv in self.odrvs:
            dump_errors(odrv, True)
            odrv.config.brake_resistance = 0.5

        for ax in (self.leftAxes + self.rightAxes):
            ax.controller.config.vel_gain = 0.01
            ax.controller.config.vel_integrator_gain = 0.05
            ax.controller.config.control_mode = 2
            ax.controller.vel_setpoint = 400
            ax.motor.config.direction = 1
            ax.sensorless_estimator.config.pm_flux_linkage = 5.51328895422 / (7 * 140) # pole pairs = 7, motor kv = 140KV

            # increase current_lim_tolerance
            ax.motor.config.current_lim_tolerance = 20
            # set to ignore illegal hall state and save all changes
            ax.encoder.config.ignore_illegal_hall_state = True

            # calibrate motor
            ax.requested_state = AXIS_STATE_MOTOR_CALIBRATION

            self.wait_and_exit_on_error(ax)

            ax.requested_state = AXIS_STATE_SENSORLESS_CONTROL
            ax.controller.vel_setpoint = 0

        # Sub to topic
        rospy.Subscriber('joy', Joy, self.vel_callback)

        # Set first watchdog
        self.timeout = timeout  # log error if this many seconds occur between received messages
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)
        self.watchdog_fired = False

        # Init other variables
        self.last_msg_time = 0
        self.last_recv_time = 0
        self.next_wd_feed_time = 0

        rospy.loginfo("Ready for topic")
        rospy.spin()

    def wait_and_exit_on_error(self,ax):
        while ax.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
            for odrv in self.odrvs:
                odrv.axis0.watchdog_feed()
                odrv.axis1.watchdog_feed()
        if ax.error != errors.axis.ERROR_NONE:
            for odrv in self.odrvs:
                if(ax == odrv.axis0 or ax == odrv.axis1):
                    dump_errors(odrv, True)

            exit()


    def get_odrives(self):
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
            done_signal.wait(timeout=120)
        finally:
            done_signal.set()

        rospy.loginfo("Found ODrives")

    def vel_callback(self, data):

        # Record times
        recv_time = rospy.Time.now().to_sec()
        msg_time = data.header.stamp.to_sec()

        # Filter old/delayed messages
        if (msg_time < (recv_time - 0.25)):
            # And if the time difference is significantly greater than the average offset
            # Ignore this msg
            rospy.logwarn("Ignoring {} second old message".format(recv_time - msg_time))
            return

        # Filter messages at frequency
        if (msg_time < (self.last_msg_time + 1.0/MSG_PER_SECOND)):
            # Ignore this message
            return
        else:
            rospy.logdebug("Time since last message received: {} seconds".format(recv_time - self.last_recv_time))
            self.last_recv_time = recv_time
            self.last_msg_time = msg_time

        # Notify of reconnection
        if (self.watchdog_fired == True):
            self.watchdog_fired = False
            self.conn_lost_dur = rospy.Time.now() - self.conn_lost_time
            rospy.logwarn("Connection to controller reestablished! Lost connection for {} seconds.".format(self.conn_lost_dur.to_sec()))

        # --- Time BEGIN here
        odrv_com_time_start = rospy.Time.now().to_sec()
        # Read errors and feed watchdog at slower rate
        if (recv_time > self.next_wd_feed_time):
            self.next_wd_feed_time = recv_time + 1.0/WD_FEED_PER_SECOND
            # Do stuff for all axes
            for ax in self.axes:
                ax.watchdog_feed()

                # TODO
                # # ODrive watchdog error clear
                # if(ax.error == errors.axis.ERROR_WATCHDOG_TIMER_EXPIRED):
                #     ax.error = errors.axis.ERROR_NONE
                #     rospy.logwarn("Cleared ODrive watchdog error")
                # For other errors
                if (ax.error != errors.axis.ERROR_NONE):
                    rospy.logfatal("Received axis error: {} {}".format(self.axes.index(ax), ax.error))
        
        # -- Time STOP: Calculate time taken to reset ODrive
        rospy.logdebug("Reseting each ODrive watchdog took {} seconds".format(rospy.Time.now().to_sec() - odrv_com_time_start))

        # Emergency brake - 4 & 5 are bumpers
        if (data.buttons[4] and data.buttons[5]):
            # Stop motors
            rospy.logdebug("Applying E-brake")
            for ax in (self.leftAxes + self.rightAxes):
                ax.controller.vel_setpoint = 0
        else:
            # Control motors as tank drive
            for ax in self.leftAxes:
                ax.controller.vel_setpoint = data.axes[1] * SPEED_LIMIT
            for ax in self.rightAxes:
                ax.controller.vel_setpoint = data.axes[4] * SPEED_LIMIT
            # -- Time STOP: Calculate time taken to reset ODrive
            rospy.logdebug("Communication with odrives took {} seconds".format(rospy.Time.now().to_sec() - odrv_com_time_start))

        rospy.loginfo(rospy.get_caller_id() + "Left: %s", data.axes[1] * SPEED_LIMIT)
        rospy.loginfo(rospy.get_caller_id() + "Right: %s", data.axes[4] * SPEED_LIMIT)

        # Received mesg so reset watchdog
        self.timer.shutdown()
        self.timer = rospy.Timer(rospy.Duration(self.timeout), self.watchdog_callback, oneshot=True)

        tot_time = rospy.Time.now().to_sec() - recv_time

        # --- Time STOP: Calculate time taken to reset ODrive
        rospy.logdebug("Callback execution took {} seconds".format(tot_time))

    def watchdog_callback(self, event):
        # Have not received mesg for self.timeout seconds
        self.conn_lost_time = rospy.Time.now()
        rospy.logwarn("Control timeout! {} seconds since last control!".format(self.timeout))
        self.watchdog_fired = True

        # Stop motors
        for ax in self.leftAxes:
            # ax.controller.vel_ramp_target = 0
            ax.controller.vel_setpoint = 0
        for ax in self.rightAxes:
            # ax.controller.vel_ramp_target = 0
            ax.controller.vel_setpoint = 0

    def clear_errors(self, odrv):
        dump_errors(odrv, True)


if __name__ == '__main__':
    rospy.init_node('driver', log_level=rospy.DEBUG)
    timeout = 2
    driver = Driver(timeout)
