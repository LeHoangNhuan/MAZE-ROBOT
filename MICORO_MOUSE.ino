#include "switch3bit.h"
#include "vl53l0x_multi.h"
#include "motor_ctrl.h"
#include "right_hand_rule.h"
#include "left_hand_rule.h"

uint8_t headingDir = 1; // 0=Bắc,1=Đông,2=Nam,3=Tây

// ===== FILTER =====
float fl_f = 0;
float l_f  = 0;
float r_f  = 0;
float fr_f = 0;

uint16_t lowpass(float &state, uint16_t val) {
  state = state * 0.6f + val * 0.4f;
  return (uint16_t)state;
}

void setup() {
  Serial.begin(115200);
  initSwitch3Bit();
  initVL53();

  // motor/encoder init
  initMotor();
  initEncoder();
  resetMoveState();

  Serial.println("MICORO_MOUSE READY");
}

void loop() {
  static uint8_t activeMode = 0;
  static bool modeLocked = false;

  if (!modeLocked) {
    activeMode = readSwitch3Bit();
    modeLocked = true;
    Serial.print("Locked mode = ");
    Serial.println(activeMode);
  }

  switch (activeMode) {

    case 0:
      Serial.println("Mode 0 - Idle");
      break;

    case 1: {
      Serial.println("Mode 1 - TEST VL53L0X");

      uint16_t fl = lowpass(fl_f, readFrontLeft());
      uint16_t l  = lowpass(l_f,  readLeft());
      uint16_t r  = lowpass(r_f,  readRight());
      uint16_t fr = lowpass(fr_f, readFrontRight());

      fr = fr - 50; // bù sai số bên phải

      Serial.print("FL: "); Serial.print(fl);
      Serial.print(" | L: "); Serial.print(l);
      Serial.print(" | R: "); Serial.print(r);
      Serial.print(" | FR: "); Serial.println(fr);
      break;
    }

    case 2: {
      Serial.println("Mode 2 - Test LEFT");
      uint16_t l = lowpass(l_f, readLeft());
      Serial.println(l);
      break;
    }

    case 3: {
      Serial.println("Mode 3 - Test RIGHT");
      uint16_t r = lowpass(r_f, readRight());
      Serial.println(r);
      break;
    }

    case 4: {
      Serial.println("Mode 4 - Test FRONT");

      uint16_t fl = lowpass(fl_f, readFrontLeft());
      uint16_t fr = lowpass(fr_f, readFrontRight());

      fr = fr - 50;

      Serial.print(fl);
      Serial.print(" | ");
      Serial.println(fr);
      break;
    }

    case 6: {
      checkMove();

      uint16_t fl = lowpass(fl_f, readFrontLeft());
      uint16_t fr = lowpass(fr_f, readFrontRight());
      uint16_t l  = lowpass(l_f,  readLeft());
      uint16_t r  = lowpass(r_f,  readRight());

      rightHandUpdate(fl, fr, l, r);
      break;
    }

    case 7: {
      checkMove();

      uint16_t fl = lowpass(fl_f, readFrontLeft());
      uint16_t fr = lowpass(fr_f, readFrontRight());
      uint16_t l  = lowpass(l_f,  readLeft());
      uint16_t r  = lowpass(r_f,  readRight());

      leftHandUpdate(fl, fr, l, r);
      break;
    }

    case 5:
      Serial.println("Mode 7");
      break;
  }

  delay(20);
}