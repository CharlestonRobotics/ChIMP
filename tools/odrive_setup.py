#!/usr/bin/python3

# Based largely on https://docs.odriverobotics.com/hoverboard.html.
# Developed and tested with ODrive Python 0.5.4.

import odrive
import argparse
import time

def configure_motor(odrv, ax_num):
    axes = [odrv.axis0, odrv.axis1]
    ax = axes[ax_num]
    ax.motor.config.pole_pairs = 15
    ax.motor.config.resistance_calib_max_voltage = 6
    ax.motor.config.requested_current_range = 25
    ax.motor.config.current_control_bandwidth = 100
    ax.motor.config.torque_constant = 1
    
def configure_encoder(odrv, ax_num):
    axes = [odrv.axis0, odrv.axis1]
    ax = axes[ax_num]
    ax.encoder.config.mode = 1 # ENCODER_MODE_HALL
    ax.encoder.config.cpr = 90
    ax.encoder.config.calib_scan_distance = 150
    ax.encoder.config.bandwidth = 100
    
def configure_controllers(odrv, ax_num):
    axes = [odrv.axis0, odrv.axis1]
    ax = axes[ax_num]
    ax.controller.config.pos_gain = 1
    torque_constant_estimate = 8
    ax.controller.config.vel_gain = 0.02 * torque_constant_estimate * ax.encoder.config.cpr
    ax.controller.config.vel_integrator_gain = 0.1 * torque_constant_estimate * ax.encoder.config.cpr
    ax.controller.config.vel_limit = 10
    ax.controller.config.control_mode = 1 # CONTROL_MODE_TORQUE_CONTROL
    
def calibrate_motor(odrv, ax_num):
    axes = [odrv.axis0, odrv.axis1]
    ax = axes[ax_num]
    print("Calibrating motor...")
    ax.requested_state = 4 # AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(1) # Wait for calibration to start.
    while ax.motor.is_armed:
        pass
    time.sleep(1)
    ax.motor.config.pre_calibrated = True
    
def calibrate_encoder(odrv, ax_num):
    axes = [odrv.axis0, odrv.axis1]
    ax = axes[ax_num]    
    print("Calibrating hall encoder polarity...")
    ax.requested_state = 12 # AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
    time.sleep(1) # Wait for calibration to start.
    while ax.motor.is_armed:
        pass
    time.sleep(1)
    print("Calibrating encoder offset...")
    ax.requested_state = 7 # AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(1) # Wait for calibration to start.
    while ax.motor.is_armed:
        pass
    time.sleep(1)
    ax.encoder.config.pre_calibrated = True

def main():
    parser = argparse.ArgumentParser(
        description="""Odrive Setup for ChIMP robot."""
    )
    parser.add_argument(
        "-a",
        metavar = "axis",
        type = int,
        required = True,
        choices = [0, 1],
        help = "Axis to be configured. Can be 0 or 1.",
    )
    args = parser.parse_args()
    ax_num = args.a
    print(f"### Configuring axis{ax_num} ###\n")
    
    print("Connecting to ODrive...")
    odrv0 = odrive.find_any()
    
    # Set DC voltage limits.
    odrv0.config.dc_bus_undervoltage_trip_level = 20
    odrv0.config.dc_bus_overvoltage_trip_level = 26
    # Set admissable battery charing current.
    odrv0.config.dc_max_negative_current = -5
    # We don't need a brake resistor since ChIMP is battery powered.
    odrv0.config.enable_brake_resistor = False
    
    # Configure GPIO for hall sensors of motor0
    odrv0.config.gpio9_mode = 0 # GPIO_MODE_DIGITAL
    odrv0.config.gpio10_mode = 0 # GPIO_MODE_DIGITAL
    odrv0.config.gpio11_mode = 0 # GPIO_MODE_DIGITAL
    
    # Configure GPIO for hall sensors of motor1
    odrv0.config.gpio3_mode = 0 # GPIO_MODE_DIGITAL
    odrv0.config.gpio4_mode = 0 # GPIO_MODE_DIGITAL
    odrv0.config.gpio5_mode = 0 # GPIO_MODE_DIGITAL

    configure_motor(odrv0, ax_num)
    configure_encoder(odrv0, ax_num)
    configure_controllers(odrv0, ax_num)
    
    # Save configuration.
    try:
        odrv0.save_configuration()
    except:
        print("Saving...")
    # Re-discover ODrive.
    odrv0 = odrive.find_any()
    try:
        odrv0.reboot()
    except:
        print("Rebooting...")
    # Re-discover ODrive.
    odrv0 = odrive.find_any()
    calibrate_motor(odrv0, ax_num)
    calibrate_encoder(odrv0, ax_num)
    
    
    
   # Save configuration.
    try:
        odrv0.save_configuration()
    except:
        print("Saving...")
    # Re-discover ODrive.
    odrv0 = odrive.find_any()
    try:
        odrv0.reboot()
    except:
        print("Rebooting...")
    # Re-discover ODrive.
    odrv0 = odrive.find_any()
    system_error = odrv0.error
    if not system_error:
        print(f"Succesfully calibrated axis{ax_num}!")
    else:
        print(f"An error has occured: odrv0.error = {system_error}.")
        
    
    
if __name__ == "__main__":
    main()

