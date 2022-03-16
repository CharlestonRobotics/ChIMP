#include <ODriveArduino.h>
#include <Command_processor.h> // https://github.com/LuSeKa/command_processor

constexpr long unsigned int kODriveBaudrate = 115200;
constexpr long unsigned int kTerminalBaudrate = 115200;

Command_processor cmd;
ODriveArduino odrive(Serial2);

void CommandMotorCurrent(float m, float current) {
  int motor = round(m);
  if (motor == 0 || motor == 1) {
    Serial.print("Motor: ");
    Serial.print(motor);
    Serial.print(", current: ");
    Serial.println(current);
    odrive.SetCurrent(motor, current);
    // odrive.run_state(motor, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  }
  else {
    Serial.println("Invalid motor selection. Must be 0 or 1.");
  }
}

void setup() {
  Serial2.begin(kODriveBaudrate);
  Serial.begin(kTerminalBaudrate);
  cmd.add_command('b', &CommandBothMotorsCurrent, 1, "Command both motors to current in mA.");
  cmd.add_command('c', &CommandMotorCurrent, 2, "Command specified current to specified motor in Amps.");
  cmd.add_command('d', &DisableMotors, 0, "Stop both motors (disablel).");
  cmd.add_command('e', &EnableMotors, 0, "Enable motors.");
  cmd.add_command('s', &StopMotors, 0, "Stop both motors (freewheel).");
}

void loop() {
  cmd.parse_command();
}

void CommandBothMotorsCurrent(float current) {
  CommandMotorCurrent(0, current);
  CommandMotorCurrent(1, current);
}

void StopMotors(int a, int b) {
  Serial.println("Stopping motors (freewheel).");
  for (int motor = 0; motor < 2; ++motor) {
    odrive.SetCurrent(motor, 0);
  }
}

void DisableMotors(int a, int b) {
  Serial.println("Stopping motors (disable).");
  for (int motor = 0; motor < 2; ++motor) {
    odrive.run_state(motor, AXIS_STATE_IDLE, false);
  }
}

void EnableMotors(int a, int b) {
  Serial.println("Enabling motors.");
  for (int motor = 0; motor < 2; ++motor) {
    odrive.run_state(motor, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
  }
}
