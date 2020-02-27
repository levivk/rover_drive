#! /usr/bin/env python

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from fibre.protocol import ChannelBrokenException
from fibre.utils import Event, Logger
import time
import argparse
import rospy

# --- DECLARATIONS --- 
ROVER_DRIVE_POLE_PAIRS = 7
ROVER_DRIVE_CPR = ROVER_DRIVE_POLE_PAIRS * 6
RAMP_RATE = 1000 # counts/sec^2

TIMEOUT = 2 # Seconds

no_calib = False

# SERIAL_NUMS = [
#     35593293288011,                  # Left, 0
#     35550393020494,                  # Middle, 1
#     35563278839886]                  # Right, 2

SERIAL_NUMS = [35550393020494]

# odrvs = [
#     None,
#     None,
#     None]

odrvs = [None]

def clear_errors(odrv):
    dump_errors(odrv, True)

def set_params(ax):

    dump_errors(ax, True)
    odrv.axis0.watchdog_feed()
    odrv.axis1.watchdog_feed()

      # --- Time BEGIN here
    readwrite_time_start = rospy.Time.now().to_sec()

    for i in range(0, 10001):
        # Either read or write 10000 times, NOT both!
        # ----- MOTOR (READ) -----
        # print("Calibrating ODrive # {}".format(to_calib.index(odrv)))
        # print("motor config.resistance_calib_max_voltage is " + str(ax.motor.config.resistance_calib_max_voltage))

        
        # # ----- ENCODER (WRITE) -----
        # print('assigning new bandwidth...')
        ax.encoder.config.bandwidth = 100
        # print("Loop Counter: " + str(i))
    
    # -- Time STOP: Calculate time taken to reset ODrive
    rospy.logdebug("Reading/Writing took {} seconds".format(rospy.Time.now().to_sec() - readwrite_time_start))



def wait_and_exit_on_error(ax):
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
        for odrv in odrvs:
            odrv.axis0.watchdog_feed()
            odrv.axis1.watchdog_feed()
    if ax.error != errors.axis.ERROR_NONE:
        for odrv in odrvs:
            if(ax == odrv.axis0 or ax == odrv.axis1):
                dump_errors(odrv, True)

        exit()

def calibrate(ax):
    # Motor calibration
    print("Calibrating motor...")
    ax.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    wait_and_exit_on_error(ax)
    print("Motor resistance: {}".format(ax.motor.config.phase_resistance))
    print("Motor inductance: {}".format(ax.motor.config.phase_inductance))

    # Encoder calibration
    print("Calibrating encoder offset...")
    ax.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    wait_and_exit_on_error(ax)
    print("Encoder offset: {}".format(ax.encoder.config.offset_float))

    # Since there have been no errors, set calibration status
    print("Saving calibration status")
    ax.motor.config.pre_calibrated = True
    ax.encoder.config.pre_calibrated = True
    ax.config.startup_closed_loop_control = True

    # # ----- WATCHDOG -----
    # # Times out during calibration, so do here.
    # ax.config.watchdog_timeout = 2


def save_reboot(odrv):
    # Save and restart
    print("Saving and rebooting")
    odrv.save_configuration()
    try:
        odrv.reboot()
    except ChannelBrokenException:
        pass


if (__name__ == "__main__"):
    
    # Parse args
    parser = argparse.ArgumentParser(description='Set odrive parameters and calibrate for rover drive')
    parser.add_argument('-a', '--axis', 
            help='Axis to calibrate', 
            dest='axis', default = 0, type=int )
    parser.add_argument('-b', '--both-axis', 
            help = 'calibrate both axis, priority over -a', 
            dest = 'calib_both_axis', 
            action = "store_true",
            default = False)
    parser.add_argument('-r', '--save-and-reboot', 
            help = 'Save and reboot odrv after complete', 
            dest = 'save_and_reboot', 
            action = "store_true",
            default = False)
    parser.add_argument('-s', '--serial-number', 
            help = 'Serial number of odrive',
            dest = 'serial_number',
            default = None, type = str )

    parser.add_argument('-w', '--which-odrive',
            help = 'Which ODrive to calibrate (1, 2, or 0). See top of this script for more info. Default is all three.',
            dest = 'which_odrive',
            default = None, type = int)

    parser.add_argument('-nc', '--no-calib',
            help = 'No calibration conducted',
            dest = 'no_calib',
            action = "store_true",
            default = False)

    args = parser.parse_args()

    print("Looking for ODrive")
    # odrv = odrive.find_any(serial_number=args.serial_number)

    # Get ODrives
    done_signal = Event(None)

    def discovered_odrv(obj):
        print("Found odrive with sn: {}".format(obj.serial_number))
        if obj.serial_number in SERIAL_NUMS:
            odrvs[SERIAL_NUMS.index(obj.serial_number)] = obj
            print("ODrive is # {}".format(SERIAL_NUMS.index(obj.serial_number)))
        else:
            print("ODrive sn not found in list. New ODrive?")
        if not None in odrvs:
            done_signal.set()

    odrive.find_all("usb", None, discovered_odrv, done_signal, None, Logger(verbose=False))
    # Wait for ODrives
    try:
        done_signal.wait(timeout=120)
    finally:
        done_signal.set()

    # Which odrives
    if args.which_odrive == None:
        to_calib = odrvs
    else:
        to_calib = [odrvs[args.which_odrive]]

    for odrv in to_calib:
        rospy.init_node('test_script', log_level=rospy.DEBUG)
        odrv.config.brake_resistance = 5.1
        print("Calibrating ODrive # {}".format(to_calib.index(odrv)))
        if args.calib_both_axis:
            odrv.axis0.watchdog_feed()
            odrv.axis1.watchdog_feed()
            clear_errors(odrv)
            set_params(odrv.axis0)
            # if not args.no_calib:
            #     calibrate(odrv.axis0)
            set_params(odrv.axis1)
            # if not args.no_calib:
            #     calibrate(odrv.axis1)
        elif args.axis == 0:
            ax = odrv.axis0
            ax.watchdog_feed()
            clear_errors(odrv)
            set_params(ax)
            # if not args.no_calib:
            #     calibrate(ax)
        elif args.axis == 1:
            ax = odrv.axis1
            ax.watchdog_feed()
            clear_errors(odrv)
            set_params(ax)
            # if not args.no_calib:
            #     calibrate(ax)
        
    if args.save_and_reboot:
        save_reboot(odrv)


# if __name__ == '__main__':
#     rospy.init_node('driver', log_level=rospy.DEBUG)
#     timeout = 2
#     driver = Driver(timeout)