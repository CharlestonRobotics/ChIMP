#include "Command_processor.h" // https://github.com/LuSeKa/command_processor

constexpr int kHallPins[3] = {A0, A1, A2};
int last_hall_state[3] = {0, 0, 0};
unsigned long counts = 0;
unsigned long illegal_counts = 0;

Command_processor cmd;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int i = 0; i < 3; ++i) {
    pinMode(kHallPins[i], INPUT_PULLUP);
  }
  cmd.add_command('r', &ResetCounters, 0, "Reset counters.");
}

void loop() {
  // put your main code here, to run repeatedly:
  int hall_state[3];
  for (int i = 0; i < 3; ++i) {
    hall_state[i] = digitalRead(kHallPins[i]);
    Serial.print(hall_state[i]);
    Serial.print('\t');
  }
  if (!IsHallStateEqual(hall_state, last_hall_state)) {
    // Hall state has changed
    if (IsHallStateIllegal(hall_state)) {
      ++illegal_counts;
    } else {
      ++counts;
    }
    CopyHallState(hall_state, last_hall_state);
  }
  Serial.print(counts);
  Serial.print('\t');
  Serial.println(illegal_counts);
  cmd.parse_command();
  delay(5);
}

bool IsHallStateEqual(int* a, int* b) {
  for (int i = 0; i < 3; ++i) {
    if(a[i] != b[i]) {
      return false;
    }
  }
  return true;
}

void CopyHallState(int* source, int* target) {
  for (int i = 0; i < 3; ++i) {
    target[i] = source[i];
  }
}

bool IsHallStateIllegal(int* state) {
  int hallSum = state[0] + state[1] + state[2];
  return (hallSum == 0 || hallSum == 3);
}

void ResetCounters(float foo, float bar) {
  counts = 0;
  illegal_counts = 0;
}
