// =============================================================================
// AUTONOMOUS WHEELCHAIR — ESP32 FIRMWARE
//
//  RECEIVE (PC → ESP32):
//     W / S / A / D   manual movement (W/S ramp up, ramp down on stop)
//     X               stop — ramps speed to 0 before stopping motors
//     B               brake — ramps to 0 then applies physical brakes
//     + / -           speed +20 / -20
//     V<nnn>\n        set speed directly, e.g. V150\n
//     M<l>,<r>\n      autonomous differential drive PWM (-255..255)
//
//  SEND (ESP32 → PC) — JSON at 50 Hz (all zeros, no sensors wired yet):
//     {"lx":0,"rx":0,"ax":0,"ay":0,"az":0,"gx":0,"gy":0,"gz":0}
//
//  PIN MAP:
//     Motor 1 (LEFT)   RPWM=25  LPWM=26
//     Motor 2 (RIGHT)  RPWM=27  LPWM=14
//     Relay 1 (LEFT  brake)  GPIO18   HIGH=release  LOW=engage
//     Relay 2 (RIGHT brake)  GPIO19
// =============================================================================

#define RPWM1 25
#define LPWM1 26
#define RPWM2 27
#define LPWM2 14

#define CH_R1 0
#define CH_L1 1
#define CH_R2 2
#define CH_L2 3

#define RELAY1 18
#define RELAY2 19

const int MAX_SPEED    = 255;
const int MIN_SPEED    =  40;   // minimum PWM for forward/backward (manual)
const int TURN_SPEED   =  30;   // in-place turn PWM (lowered for slower turns)
const int RAMP_STEP    =   8;   // PWM change per 20 ms ramp tick (~160 ms full stop)
const int MIN_AUTO_PWM =  35;   // minimum PWM for autonomous mode — below this motor stalls

int  speedVal    =  0;        // current speed magnitude for W/S
int  targetSpeed = MIN_SPEED; // desired speed (changed by +/-)
char currentCmd  = 'X';       // active command
char lastMoveDir = 'W';       // 'W' or 'S' — direction for ramp-down
bool autoMode    = false;
bool needsBrake  = false;     // apply physical brakes once ramp hits 0

unsigned long lastSendMs = 0;
const unsigned long SEND_MS = 20;

// ── hardware helpers ──────────────────────────────────────────────────────────

void releaseBrakes() { digitalWrite(RELAY1, HIGH); digitalWrite(RELAY2, HIGH); }
void applyBrakes()   { digitalWrite(RELAY1, LOW);  digitalWrite(RELAY2, LOW);  }

void setMotorPWM(int l, int r) {
  ledcWrite(CH_R1, l > 0 ?  l : 0); ledcWrite(CH_L1, l < 0 ? -l : 0);
  ledcWrite(CH_R2, r > 0 ?  r : 0); ledcWrite(CH_L2, r < 0 ? -r : 0);
}

void stopMotors() { setMotorPWM(0, 0); }

// Apply current speedVal in the last movement direction (used during ramp)
void applySpeed() {
  if (lastMoveDir == 'S') setMotorPWM(-speedVal, -speedVal);
  else                     setMotorPWM( speedVal,  speedVal);
}

// ── autonomous M command ──────────────────────────────────────────────────────

void handleM(const String& args) {
  int comma = args.indexOf(',');
  if (comma < 0) return;
  int pwm_l = constrain(args.substring(0, comma).toInt(),  -MAX_SPEED, MAX_SPEED);
  int pwm_r = constrain(args.substring(comma + 1).toInt(), -MAX_SPEED, MAX_SPEED);

  // Enforce minimum PWM so slow inner-wheel commands don't stall the motor
  if (pwm_l != 0 && abs(pwm_l) < MIN_AUTO_PWM) pwm_l = (pwm_l > 0) ? MIN_AUTO_PWM : -MIN_AUTO_PWM;
  if (pwm_r != 0 && abs(pwm_r) < MIN_AUTO_PWM) pwm_r = (pwm_r > 0) ? MIN_AUTO_PWM : -MIN_AUTO_PWM;

  if (pwm_l == 0 && pwm_r == 0) {
    stopMotors();
  } else {
    releaseBrakes();
    setMotorPWM(pwm_l, pwm_r);
  }
  autoMode = true;
  currentCmd = 'M';
}

// ── setup ─────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Wheelchair ESP32 — Motor only ===");

  pinMode(RELAY1, OUTPUT); digitalWrite(RELAY1, LOW);
  pinMode(RELAY2, OUTPUT); digitalWrite(RELAY2, LOW);

  ledcSetup(CH_R1, 1000, 8); ledcAttachPin(RPWM1, CH_R1);
  ledcSetup(CH_L1, 1000, 8); ledcAttachPin(LPWM1, CH_L1);
  ledcSetup(CH_R2, 1000, 8); ledcAttachPin(RPWM2, CH_R2);
  ledcSetup(CH_L2, 1000, 8); ledcAttachPin(LPWM2, CH_L2);
  stopMotors();

  Serial.println("Ready. Commands: W S A D X B + - V<n> M<l>,<r>");
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop() {
  // Speed ramp for manual W/S — runs every 20 ms
  static unsigned long lastRamp = 0;
  if (!autoMode && millis() - lastRamp >= 20) {
    lastRamp = millis();

    if (speedVal < targetSpeed) {
      // Ramp up
      speedVal = min(speedVal + RAMP_STEP, targetSpeed);
      if (currentCmd == 'W' || currentCmd == 'S') applySpeed();

    } else if (speedVal > targetSpeed) {
      // Ramp down (after X or B)
      speedVal = max(speedVal - RAMP_STEP, targetSpeed);
      if (speedVal == 0) {
        stopMotors();
        if (needsBrake) {
          delay(150);        // brief pause then engage brakes
          applyBrakes();
          needsBrake = false;
        }
      } else {
        applySpeed();        // decelerate in the direction we were going
      }
    }
  }

  // Serial command handling
  while (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {

      case 'W':
        autoMode = false; needsBrake = false;
        // If coming from stop/turn, reset speed so ramp starts from 0
        if (currentCmd == 'X' || currentCmd == 'A' || currentCmd == 'D') speedVal = 0;
        currentCmd = 'W'; lastMoveDir = 'W';
        targetSpeed = max(targetSpeed, MIN_SPEED);
        releaseBrakes(); applySpeed();
        break;

      case 'S':
        autoMode = false; needsBrake = false;
        if (currentCmd == 'X' || currentCmd == 'A' || currentCmd == 'D') speedVal = 0;
        currentCmd = 'S'; lastMoveDir = 'S';
        targetSpeed = max(targetSpeed, MIN_SPEED);
        releaseBrakes(); applySpeed();
        break;

      case 'A':
        autoMode = false; needsBrake = false; currentCmd = 'A';
        targetSpeed = max(targetSpeed, MIN_SPEED); speedVal = 0;
        releaseBrakes(); setMotorPWM(-TURN_SPEED, TURN_SPEED);
        break;

      case 'D':
        autoMode = false; needsBrake = false; currentCmd = 'D';
        targetSpeed = max(targetSpeed, MIN_SPEED); speedVal = 0;
        releaseBrakes(); setMotorPWM(TURN_SPEED, -TURN_SPEED);
        break;

      case 'X':
        autoMode = false; needsBrake = false;
        if (currentCmd == 'A' || currentCmd == 'D' || speedVal == 0) {
          // Turning or already stopped: immediate stop
          speedVal = 0; targetSpeed = 0; stopMotors();
        } else {
          // Moving W/S: ramp down to stop
          targetSpeed = 0;
        }
        currentCmd = 'X';
        break;

      case 'B':
        autoMode = false; needsBrake = true;
        if (currentCmd == 'A' || currentCmd == 'D' || speedVal == 0) {
          // Already slow: stop and brake immediately
          speedVal = 0; targetSpeed = 0; stopMotors();
          delay(150); applyBrakes(); needsBrake = false;
        } else {
          // Moving: ramp down, then brake (handled in ramp loop)
          targetSpeed = 0;
        }
        currentCmd = 'X';
        break;

      case '+':
        targetSpeed = min(targetSpeed + 20, MAX_SPEED);
        break;

      case '-':
        targetSpeed = max(targetSpeed - 20, MIN_SPEED);
        break;

      case 'V': {
        String s = Serial.readStringUntil('\n');
        int v = constrain(s.toInt(), MIN_SPEED, MAX_SPEED);
        targetSpeed = speedVal = v;
        applySpeed();
        break;
      }

      case 'M': {
        String args = Serial.readStringUntil('\n');
        handleM(args);
        break;
      }
    }
  }

  // JSON telemetry at 50 Hz
  if (millis() - lastSendMs >= SEND_MS) {
    lastSendMs = millis();
    Serial.println("{\"lx\":0,\"rx\":0,\"ax\":0,\"ay\":0,\"az\":0,\"gx\":0,\"gy\":0,\"gz\":0}");
  }
}
