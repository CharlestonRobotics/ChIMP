// Arduino sketch for a radio controlled twho-wheeled self balancing robot
// using a BNO055 IMU module and an ODrive motor controller. Tested on
// Arduino Mega 2560.

#include <Wire.h>
#include <Metro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ODriveArduino.h>
#include <Command_processor.h>

namespace {
// Hardware settings.
constexpr unsigned short kLedPin = 13;
constexpr unsigned short kThrottlePwmInputPin = 3;
constexpr unsigned short kSteeringPwmInputPin = 2;
constexpr unsigned short kEngagePwmInputPin = 18;
constexpr unsigned long kSerialBaudratePc = 115200;
constexpr unsigned long kSerialBaudrateOdrive = 115200;
constexpr int kMotorDir0 = 1; // Can only be 1 or -1.
constexpr int kMotorDir1 = -1; // Can only be 1 or -1.
constexpr int kSteeringPolarity = 1; // Can only be 1 or -1.
constexpr int kThrottlePolarity = 1; // Can only be 1 or -1.

// RC settings.
constexpr int kSteeringPwmOffset = 1500;
constexpr int kThrottlePwmOffset = 1350;
constexpr int kEngageThresholdPwm = 1600;

// Controller settings.
float kpBalance = 0.55;
float kdBalance = -0.045;
float kpThrottle = 0.011;
float kpSteer = 0.006;
float kdSteer = 0.01;
constexpr uint8_t kTiltDisengageThresholdDegrees = 40;
constexpr int kEngageSignalPersistenceThreshold = 2;
constexpr float kMaxAbsCurrent = 10.0;

// Task scheduling settings.
constexpr unsigned int kBlinkIntervalMs = 200;
constexpr unsigned int kControllerIntervalMs = 10;
constexpr unsigned int kActivationIntervalMs = 50;
constexpr unsigned int kPrintIntervalMs = 50;

// State variables.
bool motors_active = false; // motor state
bool tilt_limit_exceeded = false;
int engage_signal_persistence = 0;

// PWM decoder variables.
unsigned long last_throttle_pwm_rise_time = 0;
unsigned long last_steering_pwm_rise_time = 0;
unsigned long last_engage_pwm_rise_time = 0;
int throttle_pwm = 0;
int steering_pwm = 0;
int engage_pwm = 0;

// Interactive flags
bool imu_enabled = true;
bool rc_enabled = true;
bool rc_print_enabled = false;
bool motion_controller_enabled = true;

Adafruit_BNO055 bno = Adafruit_BNO055();
ODriveArduino odrive(Serial2);

// Instantiate Metros for task scheduling.
Metro led_metro = Metro(kBlinkIntervalMs);
Metro controller_metro = Metro(kControllerIntervalMs);
Metro activation_metro = Metro(kActivationIntervalMs);
Metro print_metro = Metro(kPrintIntervalMs);
Command_processor cmd;
}

void setup() {
  pinMode(kLedPin, OUTPUT);
  Serial.begin(kSerialBaudratePc); // Serial connection to PC.
  Serial2.begin(kSerialBaudrateOdrive); // Serial connection to ODrive.
  bno.setExtCrystalUse(true);
  if (!bno.begin()) {
    // BNO initialization failed. Possible reasons are 1) bad wiring or 2) incorrect I2C address.
    Serial.println("No BNO055 detected. Halting for safety.");
    while (true);
  }
  delay(1000);

  // PWM decoder interrupt handling.
  last_throttle_pwm_rise_time = micros();
  last_steering_pwm_rise_time = micros();
  last_engage_pwm_rise_time = micros();
  attachInterrupt(digitalPinToInterrupt(kSteeringPwmInputPin), SteeringPwmCallbackWrapper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kThrottlePwmInputPin), ThrottlePwmCallbackWrapper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEngagePwmInputPin), EngagePwmCallbackWrapper, CHANGE);

  // Check paramters for validity.
  bool parametrs_valid = ((abs(kMotorDir0) == 1) &&
                         (abs(kMotorDir1) == 1) &&
                         (abs(kSteeringPolarity) == 1) &&
                         (abs(kThrottlePolarity) == 1));
  if (!parametrs_valid) {
    Serial.println("Invalid parameters found. Halting for safety.");
    while (true);
  }
  cmd.add_command('p', &SetKpBalance, 1, "Set balance kp gain.");
  cmd.add_command('d', &SetKdBalance, 1, "Set balance kd gain.");
  cmd.add_command('r', &PrintControllerParameters, 0, "Print all controller parameters.");
  cmd.add_command('w', &EnableImu, 0, "Enable/disable IMU.");
  cmd.add_command('t', &EnableRcPrint, 0, "Enable/disable periodic PWM printout.");
  cmd.add_command('u', &EnableRcControl, 0, "Enable/disable RC control.");
  cmd.add_command('k', &EnableMotionController, 0, "Enable/disable motion controller.");
}

void loop() {
  if (controller_metro.check()) {
    if (motion_controller_enabled) {
      MotionController();
    }
  }
  if (print_metro.check()) {
    if (rc_print_enabled) {
      PrintRcSignals();
    }
  }
  if (activation_metro.check()) {
    // Avoid false positive disengagements due to noisy PWM by persistence filtering.
    if (engage_pwm > kEngageThresholdPwm) {
      ++ engage_signal_persistence;
    }
    else {
      -- engage_signal_persistence;
    }
    engage_signal_persistence = constrain (engage_signal_persistence, -1 * kEngageSignalPersistenceThreshold, kEngageSignalPersistenceThreshold);
    bool engage = engage_signal_persistence > 0 ? true : false;
    bool request_motor_activation = engage && !tilt_limit_exceeded;
    EngageMotors(request_motor_activation);
  }
  if (led_metro.check()) {
    digitalWrite(kLedPin, !digitalRead(kLedPin));
  }
  cmd.parse_command();
}

// Sample the IMU, compute current commands and send to the ODrive.
void MotionController() {
  // Sample the IMU.
  imu::Vector<3> euler_angles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro_rates = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float pitch = imu_enabled * euler_angles.z();
  float pitch_rate = imu_enabled * gyro_rates.x();
  float yaw_rate = imu_enabled * gyro_rates.z();

  int throttle = kThrottlePolarity * rc_enabled * (throttle_pwm - kThrottlePwmOffset);
  int steering = kSteeringPolarity * rc_enabled * (steering_pwm - kSteeringPwmOffset);

  // Update the tilt safety flag based on the latest IMU reading.
  if (abs(euler_angles.z()) > kTiltDisengageThresholdDegrees) {
    tilt_limit_exceeded = true;
  }
  else {
    tilt_limit_exceeded = false;
  }

  // Balance controller.
  float balance_controller = pitch * kpBalance + pitch_rate * kdBalance;

  // Planar motion controllers.
  float throttle_controller = kpThrottle * throttle;
  float steering_controller = kpSteer * steering + yaw_rate * kdSteer;

  float current_command_right = (balance_controller - throttle_controller - steering_controller);
  float current_command_left = (balance_controller - throttle_controller + steering_controller);

  current_command_right = constrain(current_command_right, -1 * kMaxAbsCurrent, kMaxAbsCurrent);
  current_command_left = constrain(current_command_left, -1 * kMaxAbsCurrent, kMaxAbsCurrent);

  odrive.SetCurrent(0, kMotorDir0 * current_command_right);
  odrive.SetCurrent(1, kMotorDir1 * current_command_left);
}

// Engage or disengage motors.
void EngageMotors(bool request_motors_active) {
  if (request_motors_active != motors_active) {
    switch (request_motors_active) {
      case true:
        Serial.println("Engaging motors.");
        odrive.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
        odrive.run_state(1, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
        break;
      case false:
        Serial.println("Disengaging motors.");
        odrive.run_state(0, AXIS_STATE_IDLE, false);
        odrive.run_state(1, AXIS_STATE_IDLE, false);
        break;
    }
    motors_active = request_motors_active;
  }
}

// Generic interrupt callbback function for RC PWM decoding. Measures the high pulse duration in microseconds.
void PwmInterruptCallback(unsigned long &last_rise_time, int &pwm_us, int pwm_input_pin) {
  if (digitalRead(pwm_input_pin)) { // Signal went HIGH.
    last_rise_time = micros();
  }
  else { // Signal went LOW.
    pwm_us = micros() - last_rise_time;
  }
}

// Wrappers for the three interrupt callbacks to decode the throttle, steering and engage PWM inputs.
void ThrottlePwmCallbackWrapper() {
  PwmInterruptCallback(last_throttle_pwm_rise_time, throttle_pwm, kThrottlePwmInputPin);
}

void SteeringPwmCallbackWrapper() {
  PwmInterruptCallback(last_steering_pwm_rise_time, steering_pwm, kSteeringPwmInputPin);
}

void EngagePwmCallbackWrapper() {
  PwmInterruptCallback(last_engage_pwm_rise_time, engage_pwm, kEngagePwmInputPin);
}

void PrintParameter(String name, float value) {
  Serial.print(name);
  Serial.print(":\t");
  Serial.println(value);
}

void PrintControllerParameters(float foo, float bar) {
  PrintParameter("kpBalance", kpBalance);
  PrintParameter("kdBalance", kdBalance);
  PrintParameter("kpThrottle", kpThrottle);
  PrintParameter("kpSteer", kpSteer);
  PrintParameter("kdSteer", kdSteer);
}

void PrintRcSignals() {
  Serial.print(throttle_pwm);
  Serial.print('\t');
  Serial.print(steering_pwm);
  Serial.print('\t');
  Serial.println(engage_pwm);
}

void SetKpBalance(float value, float foo) {
  Serial.println("Setting kpBalance to " + String(value));
  kpBalance = value;
}

void SetKdBalance(float value, float foo) {
  Serial.println("Setting kdBalance to " + String(value));
  kdBalance = value;
}

void EnableImu(float foo, float bar) {
  if (imu_enabled) {
    Serial.println("Disabling IMU.");
    imu_enabled = false;
  } else {
    Serial.println("Enabling IMU.");
    imu_enabled = true;
  }
}

void EnableRcControl(float foo, float bar) {
  if (rc_enabled) {
    Serial.println("Disabling RC control.");
    rc_enabled = false;
  } else {
    Serial.println("Enabling RC control.");
    rc_enabled = true;
  }
}

void EnableMotionController(float foo, float bar) {
  if (motion_controller_enabled) {
    Serial.println("Disabling motion controller.");
    motion_controller_enabled = false;
    odrive.SetCurrent(0, 0);
    odrive.SetCurrent(1, 0);
  } else {
    Serial.println("Enabling motion controller.");
    motion_controller_enabled = true;
  }
}

void EnableRcPrint(float foo, float bar) {
  if (rc_print_enabled) {
    Serial.println("Disabling PWM printout.");
    rc_print_enabled = false;
  } else {
    Serial.println("Enabling PWM printout.");
    rc_print_enabled = true;
  }
}
