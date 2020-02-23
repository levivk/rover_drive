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
RAMP_RATE = 1000 # counts/sec^2

TIMEOUT = 2 # Seconds

no_calib = False

SERIAL_NUMS = [
    35554687266894	,                  # Left, 0   (old was 35593293288011)
    35623406809166,                  # Middle, 1  (old was 35550393020494)
    35563278839886]                  # Right, 2

odrvs = [
    None,
    None,
    None]

def clear_errors(odrv):
    dump_errors(odrv, True)

def set_params(ax):

    # ----- MOTOR -----
    print('\n')
    print('----- <odrv>.<axis>.<motor> ------')
    print('assigning new pole pair #...')
    ax.motor.config.pole_pairs = ROVER_DRIVE_POLE_PAIRS
    print('assigning new resistance_calib_max_voltage...')
    ax.motor.config.resistance_calib_max_voltage = 4
    print('assigning new current range...')
    ax.motor.config.requested_current_range = 25
    ax.motor.config.current_lim = 60
    print('assigning new current control bandwidth...')
    ax.motor.config.current_control_bandwidth = 100
    
    # ----- ENCODER -----
    print('\n')
    print('----- <odrv>.<axis>.<encoder> ------')
    print('assigning new encoder mode...')
    ax.encoder.config.mode = ENCODER_MODE_HALL
    print('assigning new cpr value...')
    ax.encoder.config.cpr = ROVER_DRIVE_CPR
    print('assigning new bandwidth...')
    ax.encoder.config.bandwidth = 100
    
    # ----- CONTROLLER -----
    print('\n')
    print('----- <odrv>.<axis>.<controller> ------')
    print('assigning new position gain...')
    ax.controller.config.pos_gain = 1
    print('assigning new velocity gain...')
    ax.controller.config.vel_gain = 0.02
    print('assigning new velocity integrator gain...')
    ax.controller.config.vel_integrator_gain = 0.1
    print('assigning new velocity limit...')
    ax.controller.config.vel_limit = 1000
    print('assigning new control mode...')
    ax.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    # Ramped velocity control
    print('configuring ramped velocity control...')
    ax.controller.config.vel_ramp_rate = RAMP_RATE
    ax.controller.vel_ramp_enable = True

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
            help = 'Which ODrive to calibrate (1, 2, or 3). See top of this script for more info. Default is all three.',
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
        odrv.config.brake_resistance = 5.1
        print("Calibrating ODrive # {}".format(to_calib.index(odrv)))
        if args.calib_both_axis:
            odrv.axis0.watchdog_feed()
            odrv.axis1.watchdog_feed()
            clear_errors(odrv)
            set_params(odrv.axis0)
            if not args.no_calib:
                calibrate(odrv.axis0)
            set_params(odrv.axis1)
            if not args.no_calib:
                calibrate(odrv.axis1)
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
        
    if args.save_and_reboot:
        save_reboot(odrv)
