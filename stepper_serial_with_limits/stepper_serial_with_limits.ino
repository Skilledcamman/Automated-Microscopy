/*
stepper_serial_with_arduino_stepper_persist.ino

Serial-controlled ULN2003 + 28BYJ-48 using the Arduino Stepper library
Extended from your original sketch to:
 - Persist current_pos to EEPROM (so position survives Arduino reset/power-cycle)
 - Add a "set position" command H<n> to set the internal position counter without moving
 - Add an "E" command to force-write current_pos to EEPROM immediately
 - Write current_pos to EEPROM periodically (every EEPROM_WRITE_INTERVAL position updates) to reduce wear
 - Keep existing protocol and prints (Q still prints STEPS_PER_PRESS and other info)

Commands (all original plus new):
  U        - move "up" (positive) by STEPS_PER_PRESS steps
  D        - move "down" (negative) by STEPS_PER_PRESS steps
  P        - print current step position
  R        - release coils (de-energize motor)
  V<n>     - set speed in RPM (e.g., V8 sets 8 RPM)
  S<n>     - set STEPS_PER_PRESS to <n> (e.g., S16)
  O<n>     - select objective and its max limit: O4, O10 or O40
  M<n>     - set a custom max limit (steps) directly, e.g. M9000
  Q        - query current objective and max limit (prints useful info)
  H<n>     - set internal position counter to <n> without moving (useful to resync)
  E        - force write current_pos to EEPROM now
  ?        - print help

Notes:
 - The sketch still expects you to home manually to position 0 on first use if you want a known reference.
 - EEPROM writes are limited in frequency by an interval counter (EEPROM_WRITE_INTERVAL) to reduce wear.
 - If you want immediate persistence after every move, set EEPROM_WRITE_INTERVAL to 1 (not recommended).
*/

#include <Stepper.h>
#include <Arduino.h>
#include <EEPROM.h>

// Number of steps per revolution of your motor (common for 28BYJ-48 half-step)
const int stepsPerRevolution = 2048;

// Pins in the order: IN1 - IN3 - IN2 - IN4 (matches your working wiring)
const int PIN_IN1 = 8;
const int PIN_IN2 = 10;
const int PIN_IN3 = 9;
const int PIN_IN4 = 11;

// Create Stepper instance
Stepper myStepper = Stepper(stepsPerRevolution, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4);

// Behaviour defaults
int STEPS_PER_PRESS = 8;    // default steps per single U/D press (changeable via 'S' command)
int currentRPM = 5;         // default speed in RPM (changeable via 'V' command)
long current_pos = 0;       // open-loop step counter (home should be 0)

// Objective limits (default known values)
const long LIMIT_4X = 10000L;
const long LIMIT_10X = 10000L;
const long LIMIT_40X = 8750L;

// Current max travel limit and selected objective
long current_max_limit = LIMIT_4X;
int current_objective = 4; // 4, 10 or 40

// EEPROM addresses for persistence (use separate blocks)
const int EEPROM_ADDR_POS = 0;      // we store a long (4 bytes) at address 0
const int EEPROM_ADDR_OBJ = 8;      // store objective info starting at address 8

// EEPROM write policy: write every N position changes (reduces wear)
// Choose a reasonable number, e.g., 50 updates -> will write less frequently.
// If you want immediate persistence, set to 1 (but that's not recommended).
const unsigned long EEPROM_WRITE_INTERVAL = 50UL;
unsigned long eeprom_write_counter = 0;

// Optional: EEPROM persistence enabled
const bool ENABLE_EEPROM_PERSISTENCE = true;

// Print help text
void print_help() {
  Serial.println(F("Commands:"));
  Serial.println(F("  U        - step UP by STEPS_PER_PRESS"));
  Serial.println(F("  D        - step DOWN by STEPS_PER_PRESS"));
  Serial.println(F("  P        - print current position (steps)"));
  Serial.println(F("  R        - release / de-energize coils"));
  Serial.println(F("  V<n>     - set RPM (e.g. V8 -> 8 RPM)"));
  Serial.println(F("  S<n>     - set STEPS_PER_PRESS (e.g. S16)"));
  Serial.println(F("  O<n>     - select objective (O4, O10 or O40) and apply its max limit"));
  Serial.println(F("  M<n>     - set custom max limit (steps) directly, e.g. M9000"));
  Serial.println(F("  Q        - query current objective and max limit"));
  Serial.println(F("  H<n>     - set internal position counter to <n> (no movement)"));
  Serial.println(F("  E        - force write current_pos to EEPROM now"));
  Serial.println(F("  ?        - help"));
}

void release_coils() {
  // The Stepper library does not have a release() call, so set coil pins LOW manually.
  pinMode(PIN_IN1, OUTPUT); digitalWrite(PIN_IN1, LOW);
  pinMode(PIN_IN2, OUTPUT); digitalWrite(PIN_IN2, LOW);
  pinMode(PIN_IN3, OUTPUT); digitalWrite(PIN_IN3, LOW);
  pinMode(PIN_IN4, OUTPUT); digitalWrite(PIN_IN4, LOW);
}

// Move but enforce limits: clamp target to [0, current_max_limit]
void step_and_update(long steps) {
  long target = current_pos + steps;
  if (target < 0) target = 0;
  if (target > current_max_limit) target = current_max_limit;
  long actual = target - current_pos;
  if (actual == 0) {
    Serial.println(F("Limit reached; no movement."));
    return;
  }
  // myStepper.step expects an int; cast is safe for typical travel sizes
  myStepper.step((int)actual);
  current_pos = target;

  // update EEPROM counter and write periodically
  if (ENABLE_EEPROM_PERSISTENCE) {
    eeprom_write_counter++;
    if (eeprom_write_counter >= EEPROM_WRITE_INTERVAL) {
      EEPROM.put(EEPROM_ADDR_POS, current_pos);
      eeprom_write_counter = 0;
      Serial.print(F("Position persisted to EEPROM: "));
      Serial.println(current_pos);
    }
  }
}

// Helper to persist position immediately
void persist_position_now() {
  EEPROM.put(EEPROM_ADDR_POS, current_pos);
  eeprom_write_counter = 0;
  Serial.print(F("Position persisted to EEPROM now: "));
  Serial.println(current_pos);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; } // wait for Serial on some boards

  // Set initial speed
  myStepper.setSpeed(currentRPM);

  // Ensure pins are outputs so we can release them later
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Read persisted position/objective from EEPROM if enabled
  if (ENABLE_EEPROM_PERSISTENCE) {
    long stored_pos = 0;
    EEPROM.get(EEPROM_ADDR_POS, stored_pos);
    // Very simple sanity check; clamp stored position to known limits
    if (stored_pos < 0L) stored_pos = 0L;
    // If the stored value is larger than current_max_limit we leave it but clamp when moving
    current_pos = stored_pos;

    int stored_obj = 0;
    EEPROM.get(EEPROM_ADDR_OBJ, stored_obj);
    if (stored_obj == 4 || stored_obj == 10 || stored_obj == 40) {
      current_objective = stored_obj;
      if (current_objective == 4) current_max_limit = LIMIT_4X;
      else if (current_objective == 10) current_max_limit = LIMIT_10X;
      else if (current_objective == 40) current_max_limit = LIMIT_40X;
    }
  }

  Serial.println(F("Stepper serial control ready (with EEPROM persistence)."));
  Serial.print(F("Default STEPS_PER_PRESS=")); Serial.println(STEPS_PER_PRESS);
  Serial.print(F("Default RPM=")); Serial.println(currentRPM);
  Serial.print(F("Current objective: ")); Serial.print(current_objective); Serial.print(F("x  max_limit=")); Serial.println(current_max_limit);
  Serial.print(F("Current position (loaded/persisted): ")); Serial.println(current_pos);
  print_help();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = toupper((unsigned char)cmd.charAt(0));

  if (c == 'U') {
    // Move up: positive steps
    step_and_update(STEPS_PER_PRESS);
    Serial.print(F("Moved U. pos=")); Serial.println(current_pos);
  } else if (c == 'D') {
    // Move down: negative steps
    step_and_update(-STEPS_PER_PRESS);
    Serial.print(F("Moved D. pos=")); Serial.println(current_pos);
  } else if (c == 'P') {
    Serial.print(F("Position: ")); Serial.println(current_pos);
  } else if (c == 'R') {
    release_coils();
    Serial.println(F("Coils released."));
  } else if (c == 'V') {
    // set RPM: format V<n>
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      int v = num.toInt();
      if (v > 0) {
        currentRPM = v;
        myStepper.setSpeed(currentRPM);
        Serial.print(F("Speed set to ")); Serial.print(currentRPM); Serial.println(F(" RPM"));
      } else Serial.println(F("Invalid RPM value."));
    } else Serial.println(F("Usage: V<n> e.g. V8"));
  } else if (c == 'S') {
    // set STEPS_PER_PRESS: format S<n>
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      int s = num.toInt();
      if (s > 0) {
        STEPS_PER_PRESS = s;
        Serial.print(F("STEPS_PER_PRESS set to ")); Serial.println(STEPS_PER_PRESS);
      } else Serial.println(F("Invalid steps value."));
    } else Serial.println(F("Usage: S<n> e.g. S16"));
  } else if (c == 'O') {
    // select objective: O4, O10, O40
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      int obj = num.toInt();
      if (obj == 4) {
        current_objective = 4;
        current_max_limit = LIMIT_4X;
        Serial.println(F("Objective set to 4x."));
      } else if (obj == 10) {
        current_objective = 10;
        current_max_limit = LIMIT_10X;
        Serial.println(F("Objective set to 10x."));
      } else if (obj == 40) {
        current_objective = 40;
        current_max_limit = LIMIT_40X;
        Serial.println(F("Objective set to 40x."));
      } else {
        Serial.println(F("Unknown objective. Use O4, O10 or O40."));
      }
      Serial.print(F("Current max limit is: ")); Serial.println(current_max_limit);
      // persist objective choice
      if (ENABLE_EEPROM_PERSISTENCE) {
        EEPROM.put(EEPROM_ADDR_OBJ, current_objective);
      }
    } else {
      Serial.println(F("Usage: O<n>  e.g. O4 or O40"));
    }
  } else if (c == 'M') {
    // set custom max limit: M<n>
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      long m = num.toInt();
      if (m > 0) {
        current_max_limit = m;
        current_objective = 0; // custom
        Serial.print(F("Custom max limit set to: ")); Serial.println(current_max_limit);
      } else {
        Serial.println(F("Invalid max limit."));
      }
    } else {
      Serial.println(F("Usage: M<n> e.g. M9000"));
    }
  } else if (c == 'Q') {
    Serial.print(F("Position: ")); Serial.println(current_pos);
    Serial.print(F("Objective: "));
    if (current_objective == 0) Serial.print(F("custom"));
    else Serial.print(current_objective);
    Serial.print(F("  Max limit: ")); Serial.println(current_max_limit);
    Serial.print(F("STEPS_PER_PRESS: ")); Serial.println(STEPS_PER_PRESS);
    Serial.print(F("RPM: ")); Serial.println(currentRPM);
    Serial.print(F("EEPROM persistence: ")); Serial.println(ENABLE_EEPROM_PERSISTENCE ? "enabled" : "disabled");
    Serial.print(F("EEPROM write interval (updates): ")); Serial.println(EEPROM_WRITE_INTERVAL);
  } else if (c == 'H') {
    // set internal position counter without moving: H<n>
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      long h = num.toInt();
      if (h >= 0) {
        current_pos = h;
        // persist immediately
        if (ENABLE_EEPROM_PERSISTENCE) {
          EEPROM.put(EEPROM_ADDR_POS, current_pos);
          eeprom_write_counter = 0;
        }
        Serial.print(F("Internal position counter set to: ")); Serial.println(current_pos);
      } else {
        Serial.println(F("Invalid position value."));
      }
    } else {
      Serial.println(F("Usage: H<n> e.g. H0 (set position counter without moving)"));
    }
  } else if (c == 'E') {
    // force write current_pos to EEPROM now
    if (ENABLE_EEPROM_PERSISTENCE) {
      persist_position_now();
    } else {
      Serial.println(F("EEPROM persistence disabled."));
    }
  } else if (c == '?') {
    print_help();
  } else {
    Serial.print(F("Unknown command: ")); Serial.println(cmd);
    print_help();
  }
}