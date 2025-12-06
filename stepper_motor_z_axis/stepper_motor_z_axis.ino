/*
Serial-controlled ULN2003 + 28BYJ-48 using the Arduino Stepper library
Extended:
 - Persist current_pos to EEPROM
 - set position H<n> without moving (only allowed after homing)
 - E command to force-write (only allowed after homing)
 - periodic EEPROM writes (only after homed)
 - Added G<n> command: move exactly <n> steps (signed), with clamping and persistence
 - Added Z<n> "hard home" command: push to hard stop (ignoring limits), then optionally raise back
   and set internal position counter to 0 (and persist if enabled).

Behavior change requested:
 - On each boot the controller is UNHOMED. No movement commands that depend on position
   (U, D, G, H, E) will be accepted until a homing command (Z) is performed.
 - Position is undefined until homed. Use Z (with optional raise) to home and define pos=0.
*/

#include <Stepper.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

// Number of steps per revolution of your motor (common for 28BYJ-48 half-step)
const int stepsPerRevolution = 2048;

// Pins in the order: IN1 - IN3 - IN2 - IN4 (matches your working wiring)
const int PIN_IN1 = 8;
const int PIN_IN2 = 10;
const int PIN_IN3 = 9;
const int PIN_IN4 = 11;

// Create Stepper instance
Stepper myStepper = Stepper(stepsPerRevolution, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4);

// SoftwareSerial for external UART on dedicated pins
// Adjust these pins to your wiring if needed
const uint8_t UART_RX_PIN = 2; // connect to external TX
const uint8_t UART_TX_PIN = 3; // connect to external RX
SoftwareSerial serialPort(UART_RX_PIN, UART_TX_PIN); // RX, TX

// Behaviour defaults
int STEPS_PER_PRESS = 8;    // default steps per single U/D press (changeable via 'S' command)
int currentRPM = 5;         // default speed in RPM (changeable via 'V' command)
long current_pos = -1;      // open-loop step counter; -1 means UNHOMED / undefined

bool is_homed = false;      // whether we've performed homing since boot

// Objective limits (default known values)
const long LIMIT_4X = 9500L;
const long LIMIT_10X = 9500L;
const long LIMIT_40X = 8750L;

// Current max travel limit and selected objective
long current_max_limit = LIMIT_4X;
int current_objective = 4; // 4, 10 or 40

// EEPROM addresses for persistence (use separate blocks)
const int EEPROM_ADDR_POS = 0;      // we store a long (4 bytes) at address 0
const int EEPROM_ADDR_OBJ = 8;      // store objective info starting at address 8

// EEPROM write policy: write every N position changes (reduces wear)
const unsigned long EEPROM_WRITE_INTERVAL = 50UL;
unsigned long eeprom_write_counter = 0;

// Optional: EEPROM persistence enabled
const bool ENABLE_EEPROM_PERSISTENCE = true;

// Hard-home parameters
const long HOME_PUSH_STEPS = 10000L;     // how far to push toward the end stop (ignores limits)
// Default raise is 0 so position 0 corresponds to the physical stop.
// If you want the controller to raise after pushing, call Z<n> with a positive n.
const int HOME_RAISE_DEFAULT = 0;

// Home speed (RPM) used specifically for the homing push/raise
const int HOME_RPM = 16; // use faster speed for homing push

// Print help text
void print_help() {
  serialPort.println(F("Commands:"));
  serialPort.println(F("  Z<n>     - HARD HOME: push to end stop (ignore limits), then optionally raise by <n> steps and set pos=0"));
  serialPort.println(F("             Z without parameter will push to the stop and set pos=0 (no raise)."));
  serialPort.println(F("  Q        - query current objective and homing/state info"));
  serialPort.println(F("  ?        - help"));
  serialPort.println(F(""));
  serialPort.println(F("After homing is performed (Z), these additional commands are enabled:"));
  serialPort.println(F("  U        - step UP by STEPS_PER_PRESS"));
  serialPort.println(F("  D        - step DOWN by STEPS_PER_PRESS"));
  serialPort.println(F("  G<n>     - move exactly <n> steps (signed). e.g. G100 or G-100"));
  serialPort.println(F("  H<n>     - set internal position counter to <n> (no movement)"));
  serialPort.println(F("  E        - force write current_pos to EEPROM now"));
  serialPort.println(F(""));
  serialPort.println(F("Other config commands available anytime:"));
  serialPort.println(F("  V<n>     - set RPM (e.g. V8 -> 8 RPM)"));
  serialPort.println(F("  S<n>     - set STEPS_PER_PRESS (e.g. S16)"));
  serialPort.println(F("  O<n>     - select objective (O4, O10 or O40) and apply its max limit"));
  serialPort.println(F("  M<n>     - set custom max limit (steps) directly, e.g. M9000"));
}

// Release coils (de-energize)
void release_coils() {
  pinMode(PIN_IN1, OUTPUT); digitalWrite(PIN_IN1, LOW);
  pinMode(PIN_IN2, OUTPUT); digitalWrite(PIN_IN2, LOW);
  pinMode(PIN_IN3, OUTPUT); digitalWrite(PIN_IN3, LOW);
  pinMode(PIN_IN4, OUTPUT); digitalWrite(PIN_IN4, LOW);
}

// Move but enforce limits: clamp target to [0, current_max_limit]
// Also requires homing before moving.
void step_and_update(long steps) {
  if (!is_homed) {
    serialPort.println(F("Error: system not homed. Run Z command to perform hard home first."));
    return;
  }
  long target = current_pos + steps;
  if (target < 0) target = 0;
  if (target > current_max_limit) target = current_max_limit;
  long actual = target - current_pos;
  if (actual == 0) {
    serialPort.println(F("Limit reached; no movement."));
    return;
  }
  // myStepper.step expects an int; cast is safe for typical travel sizes
  myStepper.step((int)actual);
  current_pos = target;

  // update EEPROM counter and write periodically (only relevant after homed)
  if (ENABLE_EEPROM_PERSISTENCE && is_homed) {
    eeprom_write_counter++;
    if (eeprom_write_counter >= EEPROM_WRITE_INTERVAL) {
      EEPROM.put(EEPROM_ADDR_POS, current_pos);
      eeprom_write_counter = 0;
      serialPort.print(F("Position persisted to EEPROM: "));
      serialPort.println(current_pos);
    }
  }
}

// Helper to persist position immediately (only allowed when homed)
void persist_position_now() {
  if (!is_homed) {
    serialPort.println(F("Error: cannot persist position before homing."));
    return;
  }
  EEPROM.put(EEPROM_ADDR_POS, current_pos);
  eeprom_write_counter = 0;
  serialPort.print(F("Position persisted to EEPROM now: "));
  serialPort.println(current_pos);
}

/*
 Perform a hard home:
  - push toward the end stop by HOME_PUSH_STEPS (ignoring soft limits)
  - optionally step back up by raise_steps if raise_steps > 0
  - set current_pos = 0 and persist if enabled

 Home movement uses HOME_RPM (16) while pushing/raising, then restores the previous speed.
 After this routine completes the system is marked homed (is_homed=true) and position defined.
*/
void perform_hard_home(int raise_steps) {
  serialPort.println(F("HOMING: pushing to end stop (ignoring soft limits) ..."));

  // Save previous speed and set homing speed
  int prevSpeed = currentRPM;
  myStepper.setSpeed(HOME_RPM);

  long push = HOME_PUSH_STEPS;
  // push toward end stop (negative direction)
  myStepper.step((int)(-push)); // large push; ensure this fits in int for your board

  // small delay to allow settling
  delay(120);

  // If caller requested a positive raise, perform it. Otherwise, stay at the stop.
  if (raise_steps > 0) {
    serialPort.print(F("HOMING: raising back up by "));
    serialPort.print(raise_steps);
    serialPort.println(F(" steps (off the stop) ..."));
    myStepper.step((int)raise_steps);
    delay(60);
  } else {
    serialPort.println(F("HOMING: no raise requested; position 0 is the physical stop."));
  }

  // Restore previous speed
  myStepper.setSpeed(prevSpeed);

  // Now set internal counter to zero and persist (if enabled)
  current_pos = 0;
  is_homed = true;
  if (ENABLE_EEPROM_PERSISTENCE) {
    EEPROM.put(EEPROM_ADDR_POS, current_pos);
    eeprom_write_counter = 0;
    serialPort.print(F("Internal position counter set to: "));
    serialPort.println(current_pos);
    serialPort.println(F("Position persisted to EEPROM now."));
  } else {
    serialPort.print(F("Internal position counter set to: "));
    serialPort.println(current_pos);
  }
}

void setup() {
  serialPort.begin(9600);

  // Set initial speed
  myStepper.setSpeed(currentRPM);

  // Ensure pins are outputs so we can release them later
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // IMPORTANT: do NOT load persisted position from EEPROM on boot.
  // This ensures homing is required each boot. We still load objective choice (optional).
  if (ENABLE_EEPROM_PERSISTENCE) {
    int stored_obj = 0;
    EEPROM.get(EEPROM_ADDR_OBJ, stored_obj);
    if (stored_obj == 4 || stored_obj == 10 || stored_obj == 40) {
      current_objective = stored_obj;
      if (current_objective == 4) current_max_limit = LIMIT_4X;
      else if (current_objective == 10) current_max_limit = LIMIT_10X;
      else if (current_objective == 40) current_max_limit = LIMIT_40X;
    }
  }

  // Mark unhomed on boot
  current_pos = -1;
  is_homed = false;

  serialPort.println(F("Stepper serial control ready (SoftwareSerial @9600). SYSTEM UNHOMED - homing required on each boot."));
  serialPort.print(F("Default STEPS_PER_PRESS=")); serialPort.println(STEPS_PER_PRESS);
  serialPort.print(F("Default RPM=")); serialPort.println(currentRPM);
  serialPort.print(F("Home RPM=")); serialPort.println(HOME_RPM);
  serialPort.print(F("Current objective (loaded/persisted): ")); serialPort.print(current_objective); serialPort.print(F("x  max_limit=")); serialPort.println(current_max_limit);
  serialPort.println(F("Run Z or Z<n> to perform hard home and set position 0."));
  print_help();
}

void loop() {
  if (!serialPort.available()) return;
  String cmd = serialPort.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = toupper((unsigned char)cmd.charAt(0));

  // Always-allowed commands (config & homing)
  if (c == 'Z') {
    // HARD HOME: push to end stop (ignores soft limits), then optionally raise by <n> steps
    String num = cmd.substring(1);
    num.trim();
    int raise = HOME_RAISE_DEFAULT; // default 0 (no raise)
    if (num.length() > 0) {
      int r = num.toInt();
      if (r >= 0) raise = r;
    }
    perform_hard_home(raise);
    serialPort.println(F("Homing complete."));
    return;
  } else if (c == '?') {
    print_help();
    return;
  }

  // Other commands that are allowed regardless of homing state: V, S, O, M, Q, ?
  if (c == 'V') {
    // set RPM: format V<n>
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      int v = num.toInt();
      if (v > 0) {
        currentRPM = v;
        myStepper.setSpeed(currentRPM);
        serialPort.print(F("Speed set to ")); serialPort.print(currentRPM); serialPort.println(F(" RPM"));
      } else serialPort.println(F("Invalid RPM value."));
    } else serialPort.println(F("Usage: V<n> e.g. V8"));
    return;
  } else if (c == 'S') {
    // set STEPS_PER_PRESS: format S<n>
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      int s = num.toInt();
      if (s > 0) {
        STEPS_PER_PRESS = s;
        serialPort.print(F("STEPS_PER_PRESS set to ")); serialPort.println(STEPS_PER_PRESS);
      } else serialPort.println(F("Invalid steps value."));
    } else serialPort.println(F("Usage: S<n> e.g. S16"));
    return;
  } else if (c == 'O') {
    // select objective: O4, O10, O40
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      int obj = num.toInt();
      if (obj == 4) {
        current_objective = 4;
        current_max_limit = LIMIT_4X;
        serialPort.println(F("Objective set to 4x."));
      } else if (obj == 10) {
        current_objective = 10;
        current_max_limit = LIMIT_10X;
        serialPort.println(F("Objective set to 10x."));
      } else if (obj == 40) {
        current_objective = 40;
        current_max_limit = LIMIT_40X;
        serialPort.println(F("Objective set to 40x."));
      } else {
        serialPort.println(F("Unknown objective. Use O4, O10 or O40."));
      }
      serialPort.print(F("Current max limit is: ")); serialPort.println(current_max_limit);
      // persist objective choice
      if (ENABLE_EEPROM_PERSISTENCE) {
        EEPROM.put(EEPROM_ADDR_OBJ, current_objective);
      }
    } else {
      serialPort.println(F("Usage: O<n>  e.g. O4 or O40"));
    }
    return;
  } else if (c == 'M') {
    // set custom max limit: M<n>
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      long m = num.toInt();
      if (m > 0) {
        current_max_limit = m;
        current_objective = 0; // custom
        serialPort.print(F("Custom max limit set to: ")); serialPort.println(current_max_limit);
      } else {
        serialPort.println(F("Invalid max limit."));
      }
    } else {
      serialPort.println(F("Usage: M<n> e.g. M9000"));
    }
    return;
  } else if (c == 'Q') {
    serialPort.print(F("Homed: ")); serialPort.println(is_homed ? "yes" : "no");
    if (!is_homed) {
      serialPort.println(F("Position: UNHOMED - run Z to home and define pos=0"));
    } else {
      serialPort.print(F("Position: ")); serialPort.println(current_pos);
    }
    serialPort.print(F("Objective: "));
    if (current_objective == 0) serialPort.print(F("custom"));
    else serialPort.print(current_objective);
    serialPort.print(F("  Max limit: ")); serialPort.println(current_max_limit);
    serialPort.print(F("STEPS_PER_PRESS: ")); serialPort.println(STEPS_PER_PRESS);
    serialPort.print(F("RPM: ")); serialPort.println(currentRPM);
    serialPort.print(F("Home RPM: ")); serialPort.println(HOME_RPM);
    serialPort.print(F("EEPROM persistence: ")); serialPort.println(ENABLE_EEPROM_PERSISTENCE ? "enabled" : "disabled");
    serialPort.print(F("EEPROM write interval (updates): ")); serialPort.println(EEPROM_WRITE_INTERVAL);
    return;
  }

  // Commands that require homing
  if (!is_homed) {
    serialPort.println(F("Error: system not homed. Run Z (hard home) to set position to 0 before using movement/position commands."));
    return;
  }

  // Movement and position-related commands (only after homing)
  if (c == 'U') {
    step_and_update(STEPS_PER_PRESS);
    serialPort.print(F("Moved U. pos=")); serialPort.println(current_pos);
  } else if (c == 'D') {
    step_and_update(-STEPS_PER_PRESS);
    serialPort.print(F("Moved D. pos=")); serialPort.println(current_pos);
  } else if (c == 'P') {
    serialPort.print(F("Position: ")); serialPort.println(current_pos);
  } else if (c == 'R') {
    release_coils();
    serialPort.println(F("Coils released."));
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
        serialPort.print(F("Internal position counter set to: ")); serialPort.println(current_pos);
      } else {
        serialPort.println(F("Invalid position value."));
      }
    } else {
      serialPort.println(F("Usage: H<n> e.g. H0 (set position counter without moving)"));
    }
  } else if (c == 'E') {
    // force write current_pos to EEPROM now
    persist_position_now();
  } else if (c == 'G') {
    // Move exactly <n> steps (signed), e.g. G100 or G-50
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      long g = num.toInt(); // signed steps
      if (g != 0) {
        step_and_update(g);
        serialPort.print(F("Moved G. pos=")); serialPort.println(current_pos);
      } else {
        serialPort.println(F("G0 -> no movement."));
      }
    } else {
      serialPort.println(F("Usage: G<n> e.g. G100 or G-100 (signed steps)"));
    }
  } else {
    serialPort.print(F("Unknown command: ")); serialPort.println(cmd);
    print_help();
  }
}