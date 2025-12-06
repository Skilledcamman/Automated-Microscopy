// I2C support for Raspberry Pi host
#include <Wire.h>

// Opcodes
#define I2C_ADDR 0x12

// Opcodes
#define OP_HOME          0x01
#define OP_QUERY         0x02
#define OP_STEP_UP       0x03
#define OP_STEP_DOWN     0x04
#define OP_MOVE_STEPS    0x05
#define OP_SET_POS       0x06
#define OP_SET_OBJECTIVE 0x07
#define OP_SET_RPM       0x08
#define OP_EEPROM_WRITE  0x09

volatile uint8_t rxBuf[16];
volatile uint8_t rxLen = 0;
volatile uint8_t rxOp = 0;

// I2C helper functions mapping to existing serial-controlled logic
void doHome(int raiseSteps);
long getPosition();
long getMaxLimit();
void moveSteps(long steps);
void setPosition(long pos);
void setObjective(uint8_t code);
void setRPM(uint8_t rpm);
void eepromWritePos();
bool isHomed();

// I2C receive: [opcode][len][payload...]
void onI2CReceive(int count){
  if (count <= 0) return;
  rxOp = Wire.read();
  rxLen = Wire.read();
  for (uint8_t i=0; i<rxLen && Wire.available(); i++){
    rxBuf[i] = Wire.read();
  }
  // Execute immediately; prepare any response in global state
  switch(rxOp){
    case OP_HOME: {
      int16_t raiseSteps = (int16_t)(rxBuf[0] | (rxBuf[1]<<8));
      doHome(raiseSteps);
      break;
    }
    case OP_QUERY: {
      // no action; response will be sent in onRequest
      break;
    }
    case OP_STEP_UP: {
      int16_t s = (int16_t)(rxBuf[0] | (rxBuf[1]<<8));
      moveSteps(s);
      break;
    }
    case OP_STEP_DOWN: {
      int16_t s = (int16_t)(rxBuf[0] | (rxBuf[1]<<8));
      moveSteps(-s);
      break;
    }
    case OP_MOVE_STEPS: {
      long s = (long)( (uint32_t)rxBuf[0] | ((uint32_t)rxBuf[1]<<8) | ((uint32_t)rxBuf[2]<<16) | ((uint32_t)rxBuf[3]<<24) );
      moveSteps(s);
      break;
    }
    case OP_SET_POS: {
      long p = (long)( (uint32_t)rxBuf[0] | ((uint32_t)rxBuf[1]<<8) | ((uint32_t)rxBuf[2]<<16) | ((uint32_t)rxBuf[3]<<24) );
      setPosition(p);
      break;
    }
    case OP_SET_OBJECTIVE: {
      uint8_t code = rxBuf[0];
      setObjective(code);
      break;
    }
    case OP_SET_RPM: {
      uint8_t rpm = rxBuf[0];
      setRPM(rpm);
      break;
    }
    case OP_EEPROM_WRITE: {
      eepromWritePos();
      break;
    }
    default: break;
  }
}

// I2C response: [status][len][payload]
void onI2CRequest(){
  uint8_t status = 0;
  uint8_t payload[10];
  uint8_t len = 0;
  if (rxOp == OP_QUERY){
    // Example payload: [objective:u8][homed:u8][pos:i32][limit:i32]
    uint8_t obj = 0; // map your current objective (4x/10x/40x) to 0/1/2
    // TODO: replace with actual objective mapping
    uint8_t homed = isHomed() ? 1 : 0;
    long pos = getPosition();
    long limit = getMaxLimit();
    payload[0] = obj;
    payload[1] = homed;
    payload[2] = (uint8_t)(pos & 0xFF);
    payload[3] = (uint8_t)((pos>>8) & 0xFF);
    payload[4] = (uint8_t)((pos>>16) & 0xFF);
    payload[5] = (uint8_t)((pos>>24) & 0xFF);
    payload[6] = (uint8_t)(limit & 0xFF);
    payload[7] = (uint8_t)((limit>>8) & 0xFF);
    payload[8] = (uint8_t)((limit>>16) & 0xFF);
    payload[9] = (uint8_t)((limit>>24) & 0xFF);
    len = 10;
  }
  Wire.write(status);
  Wire.write(len);
  for (uint8_t i=0; i<len; i++) Wire.write(payload[i]);
}

// NOTE: Wire.begin and handlers are initialized in the main setup() below
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
  Serial.println(F("Commands:"));
  Serial.println(F("  Z<n>     - HARD HOME: push to end stop (ignore limits), then optionally raise by <n> steps and set pos=0"));
  Serial.println(F("             Z without parameter will push to the stop and set pos=0 (no raise)."));
  Serial.println(F("  Q        - query current objective and homing/state info"));
  Serial.println(F("  ?        - help"));
  Serial.println(F(""));
  Serial.println(F("After homing is performed (Z), these additional commands are enabled:"));
  Serial.println(F("  U        - step UP by STEPS_PER_PRESS"));
  Serial.println(F("  D        - step DOWN by STEPS_PER_PRESS"));
  Serial.println(F("  G<n>     - move exactly <n> steps (signed). e.g. G100 or G-100"));
  Serial.println(F("  H<n>     - set internal position counter to <n> (no movement)"));
  Serial.println(F("  E        - force write current_pos to EEPROM now"));
  Serial.println(F(""));
  Serial.println(F("Other config commands available anytime:"));
  Serial.println(F("  V<n>     - set RPM (e.g. V8 -> 8 RPM)"));
  Serial.println(F("  S<n>     - set STEPS_PER_PRESS (e.g. S16)"));
  Serial.println(F("  O<n>     - select objective (O4, O10 or O40) and apply its max limit"));
  Serial.println(F("  M<n>     - set custom max limit (steps) directly, e.g. M9000"));
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
    Serial.println(F("Error: system not homed. Run Z command to perform hard home first."));
    return;
  }
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

  // update EEPROM counter and write periodically (only relevant after homed)
  if (ENABLE_EEPROM_PERSISTENCE && is_homed) {
    eeprom_write_counter++;
    if (eeprom_write_counter >= EEPROM_WRITE_INTERVAL) {
      EEPROM.put(EEPROM_ADDR_POS, current_pos);
      eeprom_write_counter = 0;
      Serial.print(F("Position persisted to EEPROM: "));
      Serial.println(current_pos);
    }
  }
}

// Helper to persist position immediately (only allowed when homed)
void persist_position_now() {
  if (!is_homed) {
    Serial.println(F("Error: cannot persist position before homing."));
    return;
  }
  EEPROM.put(EEPROM_ADDR_POS, current_pos);
  eeprom_write_counter = 0;
  Serial.print(F("Position persisted to EEPROM now: "));
  Serial.println(current_pos);
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
  Serial.println(F("HOMING: pushing to end stop (ignoring soft limits) ..."));

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
    Serial.print(F("HOMING: raising back up by "));
    Serial.print(raise_steps);
    Serial.println(F(" steps (off the stop) ..."));
    myStepper.step((int)raise_steps);
    delay(60);
  } else {
    Serial.println(F("HOMING: no raise requested; position 0 is the physical stop."));
  }

  // Restore previous speed
  myStepper.setSpeed(prevSpeed);

  // Now set internal counter to zero and persist (if enabled)
  current_pos = 0;
  is_homed = true;
  if (ENABLE_EEPROM_PERSISTENCE) {
    EEPROM.put(EEPROM_ADDR_POS, current_pos);
    eeprom_write_counter = 0;
    Serial.print(F("Internal position counter set to: "));
    Serial.println(current_pos);
    Serial.println(F("Position persisted to EEPROM now."));
  } else {
    Serial.print(F("Internal position counter set to: "));
    Serial.println(current_pos);
  }
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

  Serial.println(F("Stepper serial control ready. SYSTEM UNHOMED - homing required on each boot."));
  Serial.print(F("Default STEPS_PER_PRESS=")); Serial.println(STEPS_PER_PRESS);
  Serial.print(F("Default RPM=")); Serial.println(currentRPM);
  Serial.print(F("Home RPM=")); Serial.println(HOME_RPM);
  Serial.print(F("Current objective (loaded/persisted): ")); Serial.print(current_objective); Serial.print(F("x  max_limit=")); Serial.println(current_max_limit);
  Serial.println(F("Run Z or Z<n> to perform hard home and set position 0."));
  print_help();

  // Initialize I2C interface and register handlers
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
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
    Serial.println(F("Homing complete."));
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
        Serial.print(F("Speed set to ")); Serial.print(currentRPM); Serial.println(F(" RPM"));
      } else Serial.println(F("Invalid RPM value."));
    } else Serial.println(F("Usage: V<n> e.g. V8"));
    return;
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
        Serial.print(F("Custom max limit set to: ")); Serial.println(current_max_limit);
      } else {
        Serial.println(F("Invalid max limit."));
      }
    } else {
      Serial.println(F("Usage: M<n> e.g. M9000"));
    }
    return;
  } else if (c == 'Q') {
    Serial.print(F("Homed: ")); Serial.println(is_homed ? "yes" : "no");
    if (!is_homed) {
      Serial.println(F("Position: UNHOMED - run Z to home and define pos=0"));
    } else {
      Serial.print(F("Position: ")); Serial.println(current_pos);
    }
    Serial.print(F("Objective: "));
    if (current_objective == 0) Serial.print(F("custom"));
    else Serial.print(current_objective);
    Serial.print(F("  Max limit: ")); Serial.println(current_max_limit);
    Serial.print(F("STEPS_PER_PRESS: ")); Serial.println(STEPS_PER_PRESS);
    Serial.print(F("RPM: ")); Serial.println(currentRPM);
    Serial.print(F("Home RPM: ")); Serial.println(HOME_RPM);
    Serial.print(F("EEPROM persistence: ")); Serial.println(ENABLE_EEPROM_PERSISTENCE ? "enabled" : "disabled");
    Serial.print(F("EEPROM write interval (updates): ")); Serial.println(EEPROM_WRITE_INTERVAL);
    return;
  }

  // Commands that require homing
  if (!is_homed) {
    Serial.println(F("Error: system not homed. Run Z (hard home) to set position to 0 before using movement/position commands."));
    return;
  }

  // Movement and position-related commands (only after homing)
  if (c == 'U') {
    step_and_update(STEPS_PER_PRESS);
    Serial.print(F("Moved U. pos=")); Serial.println(current_pos);
  } else if (c == 'D') {
    step_and_update(-STEPS_PER_PRESS);
    Serial.print(F("Moved D. pos=")); Serial.println(current_pos);
  } else if (c == 'P') {
    Serial.print(F("Position: ")); Serial.println(current_pos);
  } else if (c == 'R') {
    release_coils();
    Serial.println(F("Coils released."));
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
    persist_position_now();
  } else if (c == 'G') {
    // Move exactly <n> steps (signed), e.g. G100 or G-50
    String num = cmd.substring(1);
    num.trim();
    if (num.length() > 0) {
      long g = num.toInt(); // signed steps
      if (g != 0) {
        step_and_update(g);
        Serial.print(F("Moved G. pos=")); Serial.println(current_pos);
      } else {
        Serial.println(F("G0 -> no movement."));
      }
    } else {
      Serial.println(F("Usage: G<n> e.g. G100 or G-100 (signed steps)"));
    }
  } else {
    Serial.print(F("Unknown command: ")); Serial.println(cmd);
    print_help();
  }
}

// ===== I2C helper implementations mapped to existing state/behavior =====
extern int currentRPM;
extern long current_pos;
extern bool is_homed;
extern long current_max_limit;
extern int current_objective;
extern const long LIMIT_4X;
extern const long LIMIT_10X;
extern const long LIMIT_40X;
extern const bool ENABLE_EEPROM_PERSISTENCE;
extern const int EEPROM_ADDR_POS;
extern const int EEPROM_ADDR_OBJ;
extern Stepper myStepper;

void doHome(int raiseSteps){
  if (raiseSteps < 0) raiseSteps = 0;
  perform_hard_home(raiseSteps);
}

long getPosition(){
  return is_homed ? current_pos : -1;
}

long getMaxLimit(){
  return current_max_limit;
}

void moveSteps(long steps){
  // Reuse step_and_update which enforces homing and limits
  step_and_update(steps);
}

void setPosition(long pos){
  if (pos < 0) pos = 0;
  current_pos = pos;
  if (ENABLE_EEPROM_PERSISTENCE) {
    EEPROM.put(EEPROM_ADDR_POS, current_pos);
  }
}

void setObjective(uint8_t code){
  // code: 0->4x, 1->10x, 2->40x
  if (code == 0){
    current_objective = 4;
    current_max_limit = LIMIT_4X;
  } else if (code == 1){
    current_objective = 10;
    current_max_limit = LIMIT_10X;
  } else if (code == 2){
    current_objective = 40;
    current_max_limit = LIMIT_40X;
  }
  if (ENABLE_EEPROM_PERSISTENCE) {
    EEPROM.put(EEPROM_ADDR_OBJ, current_objective);
  }
}

void setRPM(uint8_t rpm){
  if (rpm == 0) rpm = 1;
  currentRPM = rpm;
  myStepper.setSpeed(currentRPM);
}

void eepromWritePos(){
  persist_position_now();
}

bool isHomed(){
  return is_homed;
}

// Update the I2C query response to reflect current objective mapping
// This function is already using current state in onI2CRequest()