// Arduino sketch for a radio controlled twho-wheeled self balancing robot
// using a BNO055 IMU module and an ODrive motor controller. Tested on
// Arduino Mega 2560.

#include <Wire.h>
#include <Metro.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ODriveArduino.h>
#include <Command_processor.h>
#include <HallEncoder.h>

namespace {
// Hardware settings.
constexpr unsigned short kLedPin = 13;
constexpr unsigned short kNeckServoPin = 4;

constexpr unsigned short kSteeringPwmInputPin = 2;
constexpr unsigned short kThrottlePwmInputPin = 3;
constexpr unsigned short kEngagePwmInputPin = 18;
constexpr unsigned short kNeckTiltPwmInputPin = 19;
// Now we are out of interrupts. Neck yaw will be controlled by the RC signal directly.
uint8_t right_hall_calibration[6] = {0,0,0,0,0,0};
uint8_t left_hall_calibration[6] = {0,0,0,0,0,0};
constexpr unsigned short kRightHallPinA = 0;
constexpr unsigned short kRightHallPinB = 0;
constexpr unsigned short kRightHallPinZ = 0;

constexpr unsigned short kLeftHallPinA = 0;
constexpr unsigned short kLeftHallPinB = 0;
constexpr unsigned short kLeftHallPinZ = 0;

constexpr unsigned long kSerialBaudratePc = 115200;
constexpr unsigned long kSerialBaudrateOdrive = 115200;
constexpr int kMotorDir0 = 1; // Can only be 1 or -1.
constexpr int kMotorDir1 = -1; // Can only be 1 or -1.
constexpr int kSteeringPolarity = 1; // Can only be 1 or -1.
constexpr int kThrottlePolarity = 1; // Can only be 1 or -1.

// RC settings.
constexpr int kSteeringPwmOffset = 1500; // Change this if your robot turns in place without steering input.
constexpr int kThrottlePwmOffset = 1350; // Change this if your robot (always) drifts forward or backward without throttle input.
constexpr int kEngageThresholdPwm = 1600;
constexpr int kNeckTiltPwmOffset = 1500;

// Controller settings.
float kpBalance = 0.55; // Refer to the /tests/readme for tuning.
float kdBalance = -0.045; // Refer to the /tests/readme for tuning.
float kpThrottle = 0.011; // Change this to control how sensitive your robot reacts to throttle input (higher value means more sensitive).
float kdThrottle = 0.0; // This parameter helps the robot to stand in place and never go too fast - like driving through honey.
float kpSteer = 0.006; // Change this to control how sensitive your robot reacts to steering input (higher value means more sensitive).
float kdSteer = 0.01; // Change this to control how well your robot tracks a straight line (higher value means it will track better, but react less to steering input).
constexpr uint8_t kTiltDisengageThresholdDegrees = 40;
constexpr int kEngageSignalPersistenceThreshold = 2;
constexpr float kMaxAbsCurrent = 10.0;
constexpr float kNeckTiltRcGain = -0.5;
constexpr float kNeckTiltImuGain = -2.0;

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
unsigned long last_neck_tilt_pwm_rise_time = 0;
int throttle_pwm = 0;
int steering_pwm = 0;
int engage_pwm = 0;
int neck_tilt_pwm = 0;

// Interactive flags
bool imu_enabled = true;
bool rc_enabled = true;
bool rc_print_enabled = false;
bool motion_controller_enabled = true;

Adafruit_BNO055 bno = Adafruit_BNO055();
ODriveArduino odrive(Serial2);
Servo neck_tilt_servo;

// Instantiate Metros for task scheduling.
Metro led_metro = Metro(kBlinkIntervalMs);
Metro controller_metro = Metro(kControllerIntervalMs);
Metro activation_metro = Metro(kActivationIntervalMs);
Metro print_metro = Metro(kPrintIntervalMs);
Command_processor cmd;
HallEncoder left_hall = HallEncoder(kLeftHallPinA, kLeftHallPinB, kLeftHallPinZ);
HallEncoder right_hall = HallEncoder(kRightHallPinA, kRightHallPinB, kRightHallPinZ);
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
  left_hall.CopyCalibration(left_hall_calibration);
  right_hall.CopyCalibration(right_hall_calibration);

  // PWM decoder interrupt handling.
  last_throttle_pwm_rise_time = micros();
  last_steering_pwm_rise_time = micros();
  last_engage_pwm_rise_time = micros();
  attachInterrupt(digitalPinToInterrupt(kSteeringPwmInputPin), SteeringPwmCallbackWrapper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kThrottlePwmInputPin), ThrottlePwmCallbackWrapper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kEngagePwmInputPin), EngagePwmCallbackWrapper, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kNeckTiltPwmInputPin), NeckTiltPwmCallbackWrapper, CHANGE);

  // Check paramters for validity.
  bool parametrs_valid = ((abs(kMotorDir0) == 1) &&
                          (abs(kMotorDir1) == 1) &&
                          (abs(kSteeringPolarity) == 1) &&
                          (abs(kThrottlePolarity) == 1));
  if (!parametrs_valid) {
    Serial.println("Invalid parameters found. Halting for safety.");
    while (true);
  }
  neck_tilt_servo.attach(kNeckServoPin);

  cmd.add_command('p', &SetKpBalance, 1, "Set balance kp gain.");
  cmd.add_command('d', &SetKdBalance, 1, "Set balance kd gain.");
  cmd.add_command('r', &PrintControllerParameters, 0, "Print all controller parameters.");
  cmd.add_command('w', &EnableImu, 0, "Enable/disable IMU.");
  cmd.add_command('t', &EnableRcPrint, 0, "Enable/disable periodic PWM printout.");
  cmd.add_command('u', &EnableRcControl, 0, "Enable/disable RC control.");
  cmd.add_command('k', &EnableMotionController, 0, "Enable/disable motion controller.");
  cmd.add_command('c', &RunHallCalibration, 1, "Run Hall encoder calibration left (0) or right (1).");
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
  // Things that should just run as often as possible.
  cmd.parse_command();
  left_hall.Update();
  right_hall.Update();
}

// Sample the IMU, compute current commands and send to the ODrive.
void MotionController() {
  // Sample the IMU.
  imu::Vector<3> euler_angles = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro_rates = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Motor controller */
  float pitch = imu_enabled * euler_angles.z();
  float pitch_rate = imu_enabled * gyro_rates.x();
  float yaw_rate = imu_enabled * gyro_rates.z();

  float linear_velocity = (left_hall.GetVelocity() + right_hall.GetVelocity())/2.0;
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
  // Throttle and steering are computed differentiy depending on the mode of operation.

  // Direct control mode
  float throttle_controller = kpThrottle * throttle - kdThrottle * linear_velocity;
  float steering_controller = kpSteer * steering + yaw_rate * kdSteer;

  // ToDo: Position hold mode.
  // set_position
  // actual_position
  // throttle_controller = kpPosition * (set_position - actual_position) - kdPosition * linear_velocity;

  float current_command_right = (balance_controller - throttle_controller - steering_controller);
  float current_command_left = (balance_controller - throttle_controller + steering_controller);

  current_command_right = constrain(current_command_right, -1 * kMaxAbsCurrent, kMaxAbsCurrent);
  current_command_left = constrain(current_command_left, -1 * kMaxAbsCurrent, kMaxAbsCurrent);

  odrive.SetCurrent(0, kMotorDir0 * current_command_right);
  odrive.SetCurrent(1, kMotorDir1 * current_command_left);

  /* Neck controller */
  int neck_tilt_output_us = kNeckTiltPwmOffset + kNeckTiltRcGain * (neck_tilt_pwm - kNeckTiltPwmOffset) + kNeckTiltImuGain * EulerToMicroseconds(pitch);
  neck_tilt_output_us = constrain(neck_tilt_output_us, 1300, 1700);
  neck_tilt_servo.writeMicroseconds(neck_tilt_output_us);
}

int EulerToMicroseconds(float euler) {
  constexpr float kMicroSecondsPerHalfRotation = 1000.0;
  return int((euler / 180.0) * kMicroSecondsPerHalfRotation);
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

void NeckTiltPwmCallbackWrapper() {
  PwmInterruptCallback(last_neck_tilt_pwm_rise_time, neck_tilt_pwm, kNeckTiltPwmInputPin);
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
  PrintParameter("kdThrottle", kdThrottle);
  PrintParameter("kpSteer", kpSteer);
  PrintParameter("kdSteer", kdSteer);
}

void PrintRcSignals() {
  Serial.print(throttle_pwm);
  Serial.print('\t');
  Serial.print(steering_pwm);
  Serial.print('\t');
  Serial.print(engage_pwm);
  Serial.print('\t');
  Serial.println(neck_tilt_pwm);
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

void RunHallCalibration(float hall_num, float foo) {
    if (hall_num == 0) {
        left_hall.Calibrate();
    }
    else if (hall_num == 1) {
        right_hall.Calibrate();
    }
    else {
        Serial.println("Invalid Hall encoder selection (must be 0 or 1).");
    }
}
