#include "motor_ctrl.h"

// ===== MOTOR PINS =====
#define IN1 PA0
#define IN2 PA1
#define IN3 PA2
#define IN4 PA3

// ===== ENCODER PINS =====
#define ENC_LEFT_A   PB8
#define ENC_LEFT_B   PB9
#define ENC_RIGHT_A  PB0
#define ENC_RIGHT_B  PB1

// ===== SPEED =====
#define BASE_SPEED_L   46//70 //(110)
#define BASE_SPEED_R   49//72 //(100)
#define SLOWDOWN_SPEED     40
#define SLOWDOWN_DIST     0

// ===== CURVE SPEED =====
#define CURVE_SPEED_OUTER  255
#define CURVE_SPEED_INNER  185

// ===== PID =====
#define PID_KP        0.4f
#define PID_KI        0.003f
#define PID_KD        0.07f
#define PID_MAX_OUT       50

// ===== DEADBAND & TIMEOUT =====
#define DEADBAND          4
#define STUCK_TIMEOUT    50

// ===== TARGET PULSES =====
#define FWD_L          1480 //
#define FWD_R          1500 //

#define SPIN90L_L      -520 //
#define SPIN90L_R      +520 //

#define SPIN180_L      -1040 //
#define SPIN180_R      +1040 //

#define SPIN90R_L      +510 //
#define SPIN90R_R      -520 //

#define CURVE_L_L   +4700 
#define CURVE_L_R   +7550 

#define CURVE_R_L   +7550 
#define CURVE_R_R   +4700

// ===== PID STATE =====
typedef struct {
  float         integral;
  float         lastError;
  unsigned long lastTime;
} PIDState;

static PIDState pidLeft;
static PIDState pidRight;

// ===== ENCODER =====
volatile long    encoderLeftCount  = 0;
volatile long    encoderRightCount = 0;
volatile uint8_t leftLastState     = 0;
volatile uint8_t rightLastState    = 0;

static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

// ===== MOVE ENGINE =====
bool moveActive    = false;
static long     targetLeft    = 0;
static long     targetRight   = 0;
static bool     isSlowingDown = false;
static bool     isCurve       = false;
static bool     curveLeft_    = false;

static long          lastProgressL = 0;
static long          lastProgressR = 0;
static unsigned long stuckTimer    = 0;
static long          lastLogPos    = 0;

static void resetPID(PIDState *pid) {
  pid->integral  = 0.0f;
  pid->lastError = 0.0f;
  pid->lastTime  = millis();
}

static float computePID(PIDState *pid, float error) {
  unsigned long now = millis();
  float dt = (now - pid->lastTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;
  pid->lastTime = now;

  float P = PID_KP * error;

  pid->integral += error * dt;
  if (pid->integral >  PID_MAX_OUT) pid->integral =  PID_MAX_OUT;
  if (pid->integral < -PID_MAX_OUT) pid->integral = -PID_MAX_OUT;
  float I = PID_KI * pid->integral;

  float D = PID_KD * (error - pid->lastError) / dt;
  pid->lastError = error;

  float out = P + I + D;
  if (out >  PID_MAX_OUT) out =  PID_MAX_OUT;
  if (out < -PID_MAX_OUT) out = -PID_MAX_OUT;
  return out;
}

// ===== ENCODER ISR =====
// LEFT: đảo dấu (+=  --> -=) vì bánh đơn quay ngược so với 2 bánh chụm
static void encoderLeftA_ISR() {
  uint8_t cur = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  encoderLeftCount -= QUAD_TABLE[(leftLastState << 2) | cur];
  leftLastState = cur;
}
static void encoderLeftB_ISR() {
  uint8_t cur = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  encoderLeftCount -= QUAD_TABLE[(leftLastState << 2) | cur];
  leftLastState = cur;
}
// RIGHT: đảo dấu (-=  --> +=) vì bánh đơn quay ngược so với 2 bánh chụm
static void encoderRightA_ISR() {
  uint8_t cur = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  encoderRightCount += QUAD_TABLE[(rightLastState << 2) | cur];
  rightLastState = cur;
}
static void encoderRightB_ISR() {
  uint8_t cur = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  encoderRightCount += QUAD_TABLE[(rightLastState << 2) | cur];
  rightLastState = cur;
}

void readEncoders(long *left, long *right) {
  noInterrupts();
  *left  = encoderLeftCount;
  *right = encoderRightCount;
  interrupts();
}

static void resetEncoders() {
  noInterrupts();
  encoderLeftCount  = 0;
  encoderRightCount = 0;
  leftLastState  = (digitalRead(ENC_LEFT_A)  << 1) | digitalRead(ENC_LEFT_B);
  rightLastState = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  interrupts();
}

// ===== MOTOR =====
// LEFT: đảo HIGH/LOW so với bản gốc
static void setMotorLeft(int speed) {
  speed = constrain(speed, -250, 250);
  if (speed > 0) {
    digitalWrite(IN2, LOW);           // đảo: gốc là HIGH
    analogWrite(IN1, speed);          // đảo: gốc là 255 - speed
  } else if (speed < 0) {
    digitalWrite(IN2, HIGH);          // đảo: gốc là LOW
    analogWrite(IN1, 255 + speed);    // đảo: gốc là -speed
  } else {
    analogWrite(IN1, 255);
    digitalWrite(IN2, LOW);
  }
}

// RIGHT: đảo HIGH/LOW so với bản gốc
static void setMotorRight(int speed) {
  speed = constrain(speed, -250, 250);
  if (speed > 0) {
    digitalWrite(IN4, LOW);           // đảo: gốc là HIGH
    analogWrite(IN3, speed);          // đảo: gốc là 255 - speed
  } else if (speed < 0) {
    digitalWrite(IN4, HIGH);          // đảo: gốc là LOW
    analogWrite(IN3, 255 + speed);    // đảo: gốc là -speed
  } else {
    analogWrite(IN3, 255);
    digitalWrite(IN4, LOW);
  }
}

void stopMotors() {
  analogWrite(IN1, 255); analogWrite(IN3, 255);
  digitalWrite(IN2, HIGH); digitalWrite(IN4, HIGH);
}

void initMotor() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();
  Serial.println("Motor init OK");
}

void initEncoder() {
  pinMode(ENC_LEFT_A,  INPUT_PULLUP);
  pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  resetEncoders();
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  encoderLeftA_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B),  encoderLeftB_ISR,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), encoderRightA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), encoderRightB_ISR, CHANGE);
  Serial.println("Encoder ISR init OK");
}

// ===== MOVE ENGINE =====
void resetMoveState() {
  lastProgressL = 0;
  lastProgressR = 0;
  stuckTimer    = millis();
  lastLogPos    = 0;
  isSlowingDown = false;
  isCurve       = false;
  curveLeft_    = false;
}

static void printMoveDone(const char *label, long lNow, long rNow, float eL, float eR) {
  Serial.print("===== "); Serial.print(label); Serial.println(" =====");
  Serial.print("L:"); Serial.print(lNow);
  Serial.print("/"); Serial.print(targetLeft);
  Serial.print(" ErrL:"); Serial.print(eL, 0);
  Serial.print(" | R:"); Serial.print(rNow);
  Serial.print("/"); Serial.print(targetRight);
  Serial.print(" ErrR:"); Serial.println(eR, 0);
}

void startMove(long leftPulses, long rightPulses) {
  Serial.print("\n>>> MOVE L:"); Serial.print(leftPulses);
  Serial.print(" R:"); Serial.println(rightPulses);

  targetLeft  = leftPulses;
  targetRight = rightPulses;
  moveActive  = true;

  resetEncoders();
  resetPID(&pidLeft);
  resetPID(&pidRight);
  resetMoveState();

  int initL = (leftPulses  >= 0) ? BASE_SPEED_L : -BASE_SPEED_L;
  int initR = (rightPulses >= 0) ? BASE_SPEED_R : -BASE_SPEED_R;

  setMotorLeft(initL);
  setMotorRight(initR);
}

void curveLeft() {
  Serial.println("\n>>> CURVE LEFT");

  targetLeft  = CURVE_L_L;
  targetRight = CURVE_L_R;
  moveActive  = true;

  resetEncoders();
  resetPID(&pidLeft);
  resetPID(&pidRight);
  resetMoveState();

  isCurve    = true;
  curveLeft_ = true;

  setMotorLeft(CURVE_SPEED_INNER);
  setMotorRight(CURVE_SPEED_OUTER);
}

void curveRight() {
  Serial.println("\n>>> CURVE RIGHT");

  targetLeft  = CURVE_R_L;
  targetRight = CURVE_R_R;
  moveActive  = true;

  resetEncoders();
  resetPID(&pidLeft);
  resetPID(&pidRight);
  resetMoveState();

  isCurve    = true;
  curveLeft_ = false;

  setMotorLeft(CURVE_SPEED_OUTER);
  setMotorRight(CURVE_SPEED_INNER);
}

void checkMove() {
  if (!moveActive) return;

  long leftNow, rightNow;
  readEncoders(&leftNow, &rightNow);

  float errL = (float)(targetLeft  - leftNow);
  float errR = (float)(targetRight - rightNow);

  long remL = abs((long)errL);
  long remR = abs((long)errR);

  long leftAbs  = abs(leftNow);
  long rightAbs = abs(rightNow);

  if (leftAbs > lastProgressL + 5 || rightAbs > lastProgressR + 5) {
    lastProgressL = leftAbs;
    lastProgressR = rightAbs;
    stuckTimer = millis();
  }

  if (millis() - stuckTimer > STUCK_TIMEOUT) {
    stopMotors();
    moveActive = false;
    printMoveDone("TIMEOUT STOP", leftNow, rightNow, errL, errR);
    resetMoveState();
    return;
  }

  if (abs(errL) <= DEADBAND && abs(errR) <= DEADBAND) {
    stopMotors();
    moveActive = false;
    printMoveDone("MOVE DONE", leftNow, rightNow, errL, errR);
    resetMoveState();
    return;
  }

  long maxRem = (remL > remR) ? remL : remR;
  if (!isSlowingDown && maxRem <= SLOWDOWN_DIST) {
    isSlowingDown = true;
    pidLeft.integral  = 0;
    pidRight.integral = 0;
    Serial.println("SLOWDOWN!");
  }

  int speedL;
  int speedR;

  if (!isSlowingDown) {
    float outL = computePID(&pidLeft,  errL);
    float outR = computePID(&pidRight, errR);

    int dirL = (errL >= 0) ? 1 : -1;
    int dirR = (errR >= 0) ? 1 : -1;

    if (isCurve) {
      int bL = curveLeft_ ? CURVE_SPEED_INNER : CURVE_SPEED_OUTER;
      int bR = curveLeft_ ? CURVE_SPEED_OUTER : CURVE_SPEED_INNER;
      speedL = (int)(dirL * bL + outL);
      speedR = (int)(dirR * bR + outR);
    } else {
      speedL = (int)(dirL * BASE_SPEED_L + outL);
      speedR = (int)(dirR * BASE_SPEED_R + outR);
    }

    speedL = constrain(speedL, -250, 250);
    speedR = constrain(speedR, -250, 250);

  } else {
    if (abs(errL) <= DEADBAND) {
      speedL = 0;
      pidLeft.integral = 0;
    } else {
      float scale = (float)remL / (float)SLOWDOWN_DIST;
      if (scale > 1.0f)  scale = 1.0f;
      if (scale < 0.15f) scale = 0.15f;
      int baseSlowL = (int)(15 + SLOWDOWN_SPEED * scale);
      int dirL = (errL > 0) ? 1 : -1;
      float outL = computePID(&pidLeft, errL);
      speedL = (int)(dirL * baseSlowL + outL * 0.15f);
      speedL = constrain(speedL, -120, 120);
    }

    if (abs(errR) <= DEADBAND) {
      speedR = 0;
      pidRight.integral = 0;
    } else {
      float scale = (float)remR / (float)SLOWDOWN_DIST;
      if (scale > 1.0f)  scale = 1.0f;
      if (scale < 0.15f) scale = 0.15f;
      int baseSlowR = (int)(15 + SLOWDOWN_SPEED * scale);
      int dirR = (errR > 0) ? 1 : -1;
      float outR = computePID(&pidRight, errR);
      speedR = (int)(dirR * baseSlowR + outR * 0.15f);
      speedR = constrain(speedR, -120, 120);
    }

    // ===== MIN SPEED CLAMP =====
    int minSpeed = 10;
    if (abs(errL) > DEADBAND && abs(speedL) < minSpeed) {
      speedL = (speedL > 0 ? minSpeed : -minSpeed);
    }
    if (abs(errR) > DEADBAND && abs(speedR) < minSpeed) {
      speedR = (speedR > 0 ? minSpeed : -minSpeed);
    }
  }

  setMotorLeft(speedL);
  setMotorRight(speedR);

  if (leftAbs - lastLogPos >= 200) {
    Serial.print("L:"); Serial.print(leftNow);
    Serial.print("/"); Serial.print(targetLeft);
    Serial.print(" R:"); Serial.print(rightNow);
    Serial.print("/"); Serial.print(targetRight);
    Serial.print(" eL:"); Serial.print(errL, 0);
    Serial.print(" eR:"); Serial.print(errR, 0);
    Serial.print(" sL:"); Serial.print(speedL);
    Serial.print(" sR:"); Serial.print(speedR);
    Serial.println(isSlowingDown ? " SLOW" : " RUN");
    lastLogPos = leftAbs;
  }
}

// ===== WRAPPER ACTIONS =====
void goForward()   { startMove(FWD_L,     FWD_R);     }
void spinLeft90()  { startMove(SPIN90L_L, SPIN90L_R); }
void spin180()     { startMove(SPIN180_L, SPIN180_R); }
void spinRight90() { startMove(SPIN90R_L, SPIN90R_R); }