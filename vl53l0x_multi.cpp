#include "vl53l0x_multi.h"

// Khai báo XSHUT
#define XSHUT_FL PA4
#define XSHUT_L  PA5
#define XSHUT_R  PA6
#define XSHUT_FR PA7

// Tạo object
VL53L0X sensorFL;
VL53L0X sensorL;
VL53L0X sensorR;
VL53L0X sensorFR;

// Địa chỉ mới
#define ADDR_FL 0x30
#define ADDR_L  0x31
#define ADDR_R  0x32
#define ADDR_FR 0x33

void initVL53() {
  Wire.setSCL(PB10);
  Wire.setSDA(PB11);
  Wire.begin();

  pinMode(XSHUT_FL, OUTPUT);
  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);
  pinMode(XSHUT_FR, OUTPUT);

  // Tắt hết sensor
  digitalWrite(XSHUT_FL, LOW);
  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_R, LOW);
  digitalWrite(XSHUT_FR, LOW);
  delay(20);

  // ===== SENSOR FL =====
  digitalWrite(XSHUT_FL, HIGH);
  delay(20);
  sensorFL.init();
  sensorFL.setAddress(ADDR_FL);

  // ===== SENSOR L =====
  digitalWrite(XSHUT_L, HIGH);
  delay(20);
  sensorL.init();
  sensorL.setAddress(ADDR_L);

  // ===== SENSOR R =====
  digitalWrite(XSHUT_R, HIGH);
  delay(20);
  sensorR.init();
  sensorR.setAddress(ADDR_R);

  // ===== SENSOR FR =====
  digitalWrite(XSHUT_FR, HIGH);
  delay(20);
  sensorFR.init();
  sensorFR.setAddress(ADDR_FR);

  // ===== TỐI ƯU (QUAN TRỌNG) =====
  sensorFL.setTimeout(50);
  sensorL.setTimeout(50);
  sensorR.setTimeout(50);
  sensorFR.setTimeout(50);

  sensorFL.setMeasurementTimingBudget(25000);
  sensorL.setMeasurementTimingBudget(25000);
  sensorR.setMeasurementTimingBudget(25000);
  sensorFR.setMeasurementTimingBudget(25000);

  // ===== START CONTINUOUS =====
  sensorFL.startContinuous();
  sensorL.startContinuous();
  sensorR.startContinuous();
  sensorFR.startContinuous();
}

// ===== Hàm đọc có chống lỗi =====
uint16_t safeRead(VL53L0X &sensor) {
  uint16_t val = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    return 2000; // giá trị fallback
  }

  if (val == 0 || val > 2000) {
    return 2000;
  }

  return val;
}

uint16_t readFrontLeft() {
  return safeRead(sensorFL);
}

uint16_t readLeft() {
  return safeRead(sensorL);
}

uint16_t readRight() {
  return safeRead(sensorR);
}

uint16_t readFrontRight() {
  return safeRead(sensorFR);
}