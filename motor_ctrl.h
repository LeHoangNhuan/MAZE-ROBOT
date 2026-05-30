#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <Arduino.h>

// ===== MOTOR/ENCODER INIT =====
void initMotor();
void initEncoder();

// ===== MOVE ENGINE =====
void startMove(long leftPulses, long rightPulses);
void curveLeft();
void curveRight();
void checkMove();
void resetMoveState();
void stopMotors();

// ===== WRAPPER ACTIONS =====
void goForward();
void spinLeft90();
void spin180();
void spinRight90();

// ===== UTIL =====
void readEncoders(long *left, long *right);

extern bool moveActive;

#endif