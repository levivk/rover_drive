#! /usr/bin/env python

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from fibre.protocol import ChannelBrokenException
from fibre.utils import Event, Logger
import time
import argparse

# --- DECLARATIONS --- 
ROVER_DRIVE_POLE_PAIRS = 7
ROVER_DRIVE_CPR = ROVER_DRIVE_POLE_PAIRS * 6
#RAMP_RATE = 1000 # counts/sec^2

TIMEOUT = 2 # Seconds

no_calib = False

SERIAL_NUMS = [
    35593293288011,                  # Left, 0   (old was 35593293288011)
    35623406809166,                  # Middle, 1  (old was 35550393020494)
    35563278839886]                  # Right, 2

odrvs = [
    None,
    None,
    None]

def clear_errors(odrv):
    dump_errors(odrv, True)

def set_params(ax):
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
    wait_and_exit_on_error(ax)

    ax.requested_state = AXIS_STATE_SENSORLESS_CONTROL
    # # Unset watchdog
    ax.config.watchdog_timeout = 0


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
    #print("Calibrating encoder offset...")
    #ax.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    #wait_and_exit_on_error(ax)
    #print("Encoder offset: {}".format(ax.encoder.config.offset_float))

    # Since there have been no errors, set calibration status
    print("Saving calibration status")
    ax.motor.config.pre_calibrated = True
    #ax.encoder.config.pre_calibrated = True
    #ax.config.startup_closed_loop_control = True
    ax.requested_state = AXIS_STATE_SENSORLESS_CONTROL
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


def get_odrives():
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
            default = None, type = int )

    parser.add_argument('-w', '--which-odrive',
            help = 'Which ODrive to calibrate (0, 1, or 2). See top of this script for more info. Default is all three.',
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

    if(args.serial_number != None):
        SERIAL_NUMS = [args.serial_number]
        odrvs = [None]

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
        odrv.config.brake_resistance = 0.5
        print("Calibrating ODrive # {}".format(to_calib.index(odrv)))
        if args.calib_both_axis:
            odrv.axis0.watchdog_feed()
            odrv.axis1.watchdog_feed()
            clear_errors(odrv)
            odrv.erase_configuration()
            # sn = hex(odrv.serial_number)
            # sn = sn[2:].upper()
            try:
                odrv.reboot()
            except ChannelBrokenException:
                pass

        elif args.axis == 0:
            ax = odrv.axis0
            ax.watchdog_feed()
            clear_errors(odrv)
            set_params(ax)
            if not args.no_calib:
                calibrate(ax)
        elif args.axis == 1:
            ax = odrv.axis1
            ax.watchdog_feed()
            clear_errors(odrv)
            set_params(ax)
            if not args.no_calib:
                calibrate(ax)
    
    print("Reconnecting to odrives")
    odrvs = [
        None,
        None,
        None]
    get_odrives()

    for odrv in odrvs:
        set_params(odrv.axis0)
        set_params(odrv.axis1)

    if args.save_and_reboot:
        save_reboot(odrv)
