#ifndef VL53_MULTI_H
#define VL53_MULTI_H

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

void initVL53();
uint16_t readFrontLeft();
uint16_t readLeft();
uint16_t readRight();
uint16_t readFrontRight();

#endif