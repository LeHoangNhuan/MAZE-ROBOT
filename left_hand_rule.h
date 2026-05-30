#ifndef LEFT_HAND_RULE_H
#define LEFT_HAND_RULE_H

#include <Arduino.h>

// Quy ước hướng: 0=Bắc, 1=Đông, 2=Nam, 3=Tây
extern uint8_t headingDir;

struct WallStateL {
  bool frontBlocked;
  bool rightOpen;
  bool leftOpen;
};

// API duy nhất dùng trong .ino
void leftHandUpdate(uint16_t fl, uint16_t fr, uint16_t left, uint16_t right);

// callback điều khiển động cơ (bạn đã có)
void goForward();
void spinRight90();
void spinLeft90();
void spin180();

#endif