// Arduino sketch for a radio controlled twho-wheeled self balancing robot
// using a BNO055 IMU module and an ODrive motor controller. Tested on
// Arduino Mega 2560.

#include <Wire.h>
#include <Metro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ODriveArduino.h>

namespace {
// Hardware settings.
constexpr unsigned short kLedPin = 13;
constexpr unsigned short kThrottlePwmInputPin = 3;
constexpr unsigned short kSteeringPwmInputPin = 2;
constexpr unsigned short kEngagePwmInputPin = 18;
constexpr unsigned long kSerialBaudratePc = 115200;
constexpr unsigned long kSerialBaudrateOdrive = 115200;
constexpr int kMotorDir0 = 1;
constexpr int kMotorDir1 = -1;

// RC settings.
constexpr int kPwmCenterValue = 1500;
constexpr int kEngageThresholdPwm = 1500;

// Controller settings.
constexpr float kKpBalance = 0.5;
constexpr float kKdBalance = -0.065;
constexpr float kKpDrive = 0.015;
constexpr float kKpSteer = 0.01;
constexpr float kKdSteer = 0.01;
constexpr uint8_t kTiltDisengageThresholdDegrees = 40;
constexpr bool kUseWheelVelocities = false;

// Task scheduling settings.
constexpr unsigned int kBlinkIntervalMs = 200;
constexpr unsigned int kControllerIntervalMs = 10;
constexpr unsigned int kActivationIntervalMs = 50;

// State flags.
bool motors_active = false; // motor state
bool tilt_limit_exceeded = false;

// PWM decoder variables.
unsigned long last_throttle_pwm_rise_time = 0;
unsigned long last_steering_pwm_rise_time = 0;
unsigned long last_engage_pwm_rise_time = 0;
int throttle_pwm = 0;
int steering_pwm = 0;
int engage_pwm = 0;

Adafruit_BNO055 bno = Adafruit_BNO055();
ODriveArduino odrive(Serial2);

// Instantiate Metros for task scheduling.
Metro led_metro = Metro(kBlinkIntervalMs);
Metro controller_metro = Metro(kControllerIntervalMs);
Metro activation_metro = Metro(kActivationIntervalMs);
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
  if (!((kMotorDir0 == 1 || kMotorDir0 == -1) && (kMotorDir1 == 1 || kMotorDir1 == -1))) {
    Serial.println("Invalid motor direction paramters found. Halting for safety.");
    while (true);
  }
  Serial.println("Starting ChIMP!");
}

void loop() {
  if (controller_metro.check()) {
    MotionController();
  }
  if (activation_metro.check()) {
    bool request_motor_activation = engage_pwm > kEngageThresholdPwm && !tilt_limit_exceeded;
    EngageMotors(request_motor_activation);
  }
  if (led_metro.check()) {
    digitalWrite(kLedPin, !digitalRead(kLedPin));
  }
}

// Sample the IMU, compute current commands and send to the ODrive.
void MotionController() {
  // Sample the IMU.
  imu::Vector<3> euler_angles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro_rates = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Update the tilt safety flag based on the latest IMU reading.
  if (abs(euler_angles.z()) > kTiltDisengageThresholdDegrees) {
    tilt_limit_exceeded = true;
  }
  else {
    tilt_limit_exceeded = false;
  }

  // Balance controller.
  float balance_controller = euler_angles.z() * kKpBalance + gyro_rates.x() * kKdBalance;

  // Planar motion controllers.
  float planar_velocity = 0;
  if (kUseWheelVelocities) {
    // Get wheel velocities. This generates a lot of blocking serial traffic and might slow down the control loop.
    float wheel_velocity_right = odrive.GetVelocity(0);
    float wheel_velocity_left = odrive.GetVelocity(1);
    planar_velocity = (wheel_velocity_right + wheel_velocity_left) / 2.0;
  }
  //TODO(LuSeKa): Scale the position controller output based on the planar velocity (if feasible).
  float position_controller = kKpDrive * (throttle_pwm - kPwmCenterValue);
  float steering_controller = kKpSteer * (steering_pwm - kPwmCenterValue) + gyro_rates.z() * kKdSteer;

  float current_command_right = (balance_controller - position_controller - steering_controller);
  float current_command_left = (balance_controller - position_controller + steering_controller);

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
