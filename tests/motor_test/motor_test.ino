#include <ODriveArduino.h>
#include <Command_processor.h> // https://github.com/LuSeKa/command_processor

constexpr long unsigned int kODriveBaudrate = 230400;
constexpr long unsigned int kTerminalBaudrate = 115200;

Command_processor cmd;
ODriveArduino odrive(Serial2); // instantiate ODrive

void CommandMotorCurrent(int motor, int current) {
  if (motor == 0 || motor == 1) {
    float current_in_Amps = (float)current / 1000.0;
    Serial.print("Motor: ");
    Serial.print(motor);
    Serial.print(", current: ");
    Serial.println(current_in_Amps);
    odrive.SetCurrent(motor, current_in_Amps);
    odrive.run_state(motor, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  }
  else {
    Serial.println("Invalid motor selection. Must be 0 or 1.");
  }
}

void CommandBothMotorsCurrent(int current) {
  CommandMotorCurrent(0, current);
  CommandMotorCurrent(1, current);
}

void StopMotors(int a, int b) {
  for (int motor = 0; motor < 2; ++motor) {
    odrive.SetCurrent(motor, 0);
    odrive.run_state(motor, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  }
}

void setup() {
  Serial2.begin(kODriveBaudrate);
  Serial.begin(kTerminalBaudrate);
  odrive.SetCurrent(0, 0);
  odrive.SetCurrent(1, 0);
  Serial.println("Engaging motors");
  cmd.add_command('c', &CommandMotorCurrent, 2, "Command specified current to specified motor in mA.");
  cmd.add_command('s', &StopMotors, 0, "Stop both motors (disable).");
  cmd.add_command('b', &CommandBothMotorsCurrent, 1, "Command both motors to current in mA.");
}

void loop() {
  cmd.parse_command();
}
