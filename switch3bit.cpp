#include "switch3bit.h"
#include "vl53l0x_multi.h"

// Khai báo chân (STM32)
#define BIT0 PA8
#define BIT1 PA11
#define BIT2 PA12

void initSwitch3Bit() {
  pinMode(BIT0, INPUT_PULLUP);
  pinMode(BIT1, INPUT_PULLUP);
  pinMode(BIT2, INPUT_PULLUP);
}

// Hàm đọc và ghép 3 bit
uint8_t readSwitch3Bit() {
  uint8_t b0 = !digitalRead(BIT0);
  uint8_t b1 = !digitalRead(BIT1);
  uint8_t b2 = !digitalRead(BIT2);

  return (b2 << 2) | (b1 << 1) | b0;
}