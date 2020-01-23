#! /usr/bin/env python

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
from fibre.protocol import ChannelBrokenException
import time
import argparse

# --- DECLARATIONS --- 
ROVER_DRIVE_POLE_PAIRS = 7
ROVER_DRIVE_CPR = ROVER_DRIVE_POLE_PAIRS * 6
RAMP_RATE = 500 # counts/sec^2

TIMEOUT = 2 # Seconds

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
    ax.controller.config.vel_ramp_rate = 500
    ax.controller.vel_ramp_enable = True

    # Unset watchdog
    ax.config.watchdog_timeout = 0


def wait_and_exit_on_error(ax):
    while ax.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    if ax.error != errors.axis.ERROR_NONE:
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

    # ----- WATCHDOG -----
    # Times out during calibration, so do here.
    ax.config.watchdog_timeout = 2


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

    args = parser.parse_args()

    print("Looking for ODrive")
    odrv = odrive.find_any()
    print("Found ODrive")

    if args.calib_both_axis:
        clear_errors(odrv)
        set_params(odrv.axis0)
        calibrate(odrv.axis0)
        set_params(odrv.axis1)
        calibrate(odrv.axis1)
    elif args.axis == 0:
        clear_errors(odrv)
        set_params(odrv.axis0)
        calibrate(odrv.axis0)
    elif args.axis == 1:
        clear_errors(odrv)
        set_params(odrv.axis1)
        calibrate(odrv.axis1)
    
    
    if args.save_and_reboot:
        save_reboot(odrv)