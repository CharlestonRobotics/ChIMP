#!/usr/bin/python3

# Based largely on https://docs.odriverobotics.com/v/latest/hoverboard.html
# Developed and tested with ODrive Python 0.5.4. and ODrive firmware version 0.5.4

import argparse
import odrive
import sys
import time

def configure_motor(ax):
    ax.motor.config.pole_pairs = 15
    ax.motor.config.resistance_calib_max_voltage = 6
    ax.motor.config.requested_current_range = 25
    ax.motor.config.current_control_bandwidth = 100
    ax.motor.config.torque_constant = 1
    ax.motor.config.current_lim = 15


def configure_encoder(ax):
    ax.encoder.config.mode = 1 # ENCODER_MODE_HALL
    ax.encoder.config.cpr = 90
    ax.encoder.config.calib_scan_distance = 150
    ax.encoder.config.bandwidth = 100


def configure_controllers(ax):
    ax.controller.config.pos_gain = 1
    torque_constant_estimate = 8
    ax.controller.config.vel_gain = 0.02 * torque_constant_estimate * ax.encoder.config.cpr
    ax.controller.config.vel_integrator_gain = 0.1 * torque_constant_estimate * ax.encoder.config.cpr
    ax.controller.config.vel_limit = 10
    ax.controller.config.control_mode = 1 # CONTROL_MODE_TORQUE_CONTROL
    ax.controller.config.enable_torque_mode_vel_limit = False


def encoder_error_to_string(error):
    if error == odrive.enums.ENCODER_ERROR_ABS_SPI_COM_FAIL:
        return "ENCODER_ERROR_ABS_SPI_COM_FAIL"
    elif error == odrive.enums.ENCODER_ERROR_ABS_SPI_NOT_READY:
        return "ENCODER_ERROR_ABS_SPI_NOT_READY"
    elif error == odrive.enums.ENCODER_ERROR_ABS_SPI_TIMEOUT:
        return "ENCODER_ERROR_ABS_SPI_TIMEOUT"
    elif error == odrive.enums.ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH:
        return "ENCODER_ERROR_CPR_POLEPAIRS_MISMATCH"
    elif error == odrive.enums.ENCODER_ERROR_HALL_NOT_CALIBRATED_YET:
        return "ENCODER_ERROR_HALL_NOT_CALIBRATED_YET"
    elif error == odrive.enums.ENCODER_ERROR_ILLEGAL_HALL_STATE:
        return "ENCODER_ERROR_ILLEGAL_HALL_STATE"
    elif error == odrive.enums.ENCODER_ERROR_INDEX_NOT_FOUND_YET:
        return "ENCODER_ERROR_INDEX_NOT_FOUND_YET"
    elif error == odrive.enums.ENCODER_ERROR_NONE:
        return "ENCODER_ERROR_NONE"
    elif error == odrive.enums.ENCODER_ERROR_NO_RESPONSE:
        return "ENCODER_ERROR_NO_RESPONSE"
    elif error == odrive.enums.ENCODER_ERROR_UNSTABLE_GAIN:
        return "ENCODER_ERROR_UNSTABLE_GAIN"
    elif error == odrive.enums.ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE:
        return "ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE"
    else:
        return f"Unknown encoder error code: {error}"


def motor_error_to_string(error):
    if error == odrive.enums.MOTOR_ERROR_BAD_TIMING:
        return "MOTOR_ERROR_BAD_TIMING"
    elif error == odrive.enums.MOTOR_ERROR_BRAKE_RESISTOR_DISARMED:
        return "MOTOR_ERROR_BRAKE_RESISTOR_DISARMED"
    elif error == odrive.enums.MOTOR_ERROR_CONTROLLER_FAILED:
        return "MOTOR_ERROR_CONTROLLER_FAILED"
    elif error == odrive.enums.MOTOR_ERROR_CONTROLLER_INITIALIZING:
        return "MOTOR_ERROR_CONTROLLER_INITIALIZING"
    elif error == odrive.enums.MOTOR_ERROR_CONTROL_DEADLINE_MISSED:
        return "MOTOR_ERROR_CONTROL_DEADLINE_MISSED"
    elif error == odrive.enums.MOTOR_ERROR_CURRENT_LIMIT_VIOLATION:
        return "MOTOR_ERROR_CURRENT_LIMIT_VIOLATION"
    elif error == odrive.enums.MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE:
        return "MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE"
    elif error == odrive.enums.MOTOR_ERROR_CURRENT_SENSE_SATURATION:
        return "MOTOR_ERROR_CURRENT_SENSE_SATURATION"
    elif error == odrive.enums.MOTOR_ERROR_DRV_FAULT:
        return "MOTOR_ERROR_DRV_FAULT"
    elif error == odrive.enums.MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP:
        return "MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP"
    elif error == odrive.enums.MOTOR_ERROR_I_BUS_OUT_OF_RANGE:
        return "MOTOR_ERROR_I_BUS_OUT_OF_RANGE"
    elif error == odrive.enums.MOTOR_ERROR_MODULATION_IS_NAN:
        return "MOTOR_ERROR_MODULATION_IS_NAN"
    elif error == odrive.enums.MOTOR_ERROR_MODULATION_MAGNITUDE:
        return "MOTOR_ERROR_MODULATION_MAGNITUDE"
    elif error == odrive.enums.MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP:
        return "MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP"
    elif error == odrive.enums.MOTOR_ERROR_NONE:
        return "MOTOR_ERROR_NONE"
    elif error == odrive.enums.MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE:
        return "MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE"
    elif error == odrive.enums.MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE:
        return "MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE"
    elif error == odrive.enums.MOTOR_ERROR_SYSTEM_LEVEL:
        return "MOTOR_ERROR_SYSTEM_LEVEL"
    elif error == odrive.enums.MOTOR_ERROR_TIMER_UPDATE_MISSED:
        return "MOTOR_ERROR_TIMER_UPDATE_MISSED"
    elif error == odrive.enums.MOTOR_ERROR_UNBALANCED_PHASES:
        return "MOTOR_ERROR_UNBALANCED_PHASES"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND:
        return "MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT:
        return "MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_GAINS:
        return "MOTOR_ERROR_UNKNOWN_GAINS"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE:
        return "MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_PHASE_VEL:
        return "MOTOR_ERROR_UNKNOWN_PHASE_VEL"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_TORQUE:
        return "MOTOR_ERROR_UNKNOWN_TORQUE"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE:
        return "MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE"
    elif error == odrive.enums.MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND:
        return "MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND"
    else:
        return f"Unknown motor error code: {error}"


def odrive_error_to_string(error):
    if error == odrive.enums.ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION:
        return "ODRIVE_ERROR_BRAKE_DEADTIME_VIOLATION"
    elif error == odrive.enums.ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN:
        return "ODRIVE_ERROR_BRAKE_DUTY_CYCLE_NAN"
    elif error == odrive.enums.ODRIVE_ERROR_CONTROL_ITERATION_MISSED:
        return "ODRIVE_ERROR_CONTROL_ITERATION_MISSED"
    elif error == odrive.enums.ODRIVE_ERROR_DC_BUS_OVER_CURRENT:
        return "ODRIVE_ERROR_DC_BUS_OVER_CURRENT"
    elif error == odrive.enums.ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT:
        return "ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT"
    elif error == odrive.enums.ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE:
        return "ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE"
    elif error == odrive.enums.ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE:
        return "ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE"
    elif error == odrive.enums.ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE:
        return "ODRIVE_ERROR_INVALID_BRAKE_RESISTANCE"
    elif error == odrive.enums.ODRIVE_ERROR_NONE:
        return "ODRIVE_ERROR_NONE"
    else:
        return f"Unknown odrive error code: {error}"


def calibrate_motor(ax):
    print("Calibrating motor...")
    ax.requested_state = 4 # AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(1) # Wait for calibration to start.
    while ax.motor.is_armed:
        pass
    time.sleep(1)
    phase_inductance = ax.motor.config.phase_inductance
    phase_resistance = ax.motor.config.phase_resistance
    print(f"Motor calibration result:\n\tPhase inductance: {phase_inductance}\n\tPhase resistance: {phase_resistance}")
    return ax.motor.error


def calibrate_encoder_polarity(ax):
    print("Calibrating hall encoder polarity...")
    ax.requested_state = 12 # AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
    time.sleep(1) # Wait for calibration to start.
    while ax.motor.is_armed:
        pass
    time.sleep(1)
    return ax.encoder.error


def calibrate_encoder_offset(ax):
    print("Calibrating encoder offset...")
    ax.requested_state = 7 # AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    time.sleep(1) # Wait for calibration to start.
    while ax.motor.is_armed:
        pass
    time.sleep(1)
    phase_offset = ax.encoder.config.phase_offset_float
    print(f"Encoder calibration result:\n\tPhase offset: {phase_offset}")
    return ax.encoder.error


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
    axes = [odrv0.axis0, odrv0.axis1]
    ax = axes[ax_num]
    print("Setting parameters...")
    # Configure serial baudrate.
    odrv0.config.uart_a_baudrate = 115200
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

    configure_motor(ax)
    configure_encoder(ax)
    configure_controllers(ax)
    
    # Save configuration.
    try:
        odrv0.save_configuration()
    except:
        print("Save and reboot...")
    # Re-discover ODrive.
    odrv0 = odrive.find_any()
    axes = [odrv0.axis0, odrv0.axis1]
    ax = axes[ax_num]
    motor_error = calibrate_motor(ax)
    if motor_error:
        sys.exit(f"Motor calibration failed with motor error {motor_error_to_string(motor_error)}.")
    ax.motor.config.pre_calibrated = True

    encoder_error = calibrate_encoder_polarity(ax)
    if encoder_error:
        sys.exit(f"Encoder polarity calibration failed with encoder error {encoder_error_to_string(encoder_error)}.")

    encoder_error = calibrate_encoder_offset(ax)
    if encoder_error:
        sys.exit(f"Encoder offset calibration failed with encoder error {encoder_error_to_string(encoder_error)}.")

    ax.encoder.config.pre_calibrated = True

    # Save configuration.
    try:
        odrv0.save_configuration()
    except:
        print("Save and reboot...")
    # Re-discover ODrive.
    odrv0 = odrive.find_any()
    system_error = odrv0.error
    if system_error:
        sys.exit(f"Calibration failed with system error {odrive_error_to_string(system_error)}.")
    print(f"Succesfully calibrated axis{ax_num}!")
    sys.exit(0)


if __name__ == "__main__":
    main()

