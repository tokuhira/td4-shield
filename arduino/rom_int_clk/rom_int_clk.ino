/**
   ROM emulator for TD4 using interrupt on CLK
*/
#include "td4_iface.h"
#include "td4_rom.h"

#define USE_PCA9624 // Drive 7 segments LED by PCA9624PW with program counter.

#ifdef USE_PCA9624
#include <Wire.h>
#include <string.h>
#include "pca9624.h"

typedef unsigned char seg7[7];
seg7 hex2seg7[] = { // 0xff means PWM output 99.6%
  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00 }, // 0
  { 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00 }, // 1
  { 0xff, 0xff, 0x00, 0xff, 0xff, 0x00, 0xff }, // 2
  { 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff }, // 3
  { 0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff }, // 4
  { 0xff, 0x00, 0xff, 0xff, 0x00, 0xff, 0xff }, // 5
  { 0xff, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff }, // 6
  { 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00 }, // 7
  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }, // 8
  { 0xff, 0xff, 0xff, 0xff, 0x00, 0xff, 0xff }, // 9
  { 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff }, // a
  { 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff }, // b
  { 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0xff }, // c
  { 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0xff }, // d
  { 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0xff }, // e
  { 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff }, // f
};
#endif

const uint32_t clock_pin = CLK;
const uint32_t base_address = 0;
const uint32_t default_address = 0;

volatile int ticked = false;

void tick() {
  ticked = true;
}

#ifdef USE_PCA9624
uint8_t i2cWrite(char slave_address, unsigned char *cmd, int num) {
  Wire.beginTransmission(slave_address);
  for (int i = 0; i < num; i++) {
    Wire.write(*(cmd + i));
  }
  return Wire.endTransmission();
}

void writeSeg7(seg7 data) {
  unsigned char cmd[9];
  cmd[0] = PWM0 | 0x80;     // PWM0 + auto increment
  memcpy(&cmd[1], data, 7);
  cmd[8] = 0xff;            // Light on decimal point
  i2cWrite(PCA9624_ADDR, cmd, 9);
}

void writeSegDP(bool light) {
  unsigned char cmd[2];
  cmd[0] = PWM7;            // PWM7
  cmd[1] = light ? 0xff : 0x00;
  i2cWrite(PCA9624_ADDR, cmd, 2);
}

void initLEDdriver() {
  unsigned char cmd[3];

  cmd[0] = MODE1;           // MODE1
  cmd[1] = 0x00;            // SLEEP = 0
  i2cWrite(PCA9624_ADDR, cmd, 2);

  cmd[0] = LEDOUT0 | 0x80;  // LEDOUT0 + auto increment
  cmd[1] = 0xAA;            // LED 3, 2, 1, 0 : PWM(=10)
  cmd[2] = 0xAA;            // LED 7, 6, 5, 4 : PWM(=10)
  i2cWrite(PCA9624_ADDR, cmd, 3);
}
#endif

void setup() {
  for (uint32_t i = in_0; i < in_count; ++i) {
    pinMode(in_pins[i], INPUT);
  }
  for (uint32_t i = out_0; i < out_count; ++i) {
    pinMode(out_pins[i], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(clock_pin), tick, RISING);

#ifdef USE_PCA9624
  Wire.begin();
  initLEDdriver();
#endif
}

void loop() {
  static uint32_t address = default_address;

  if (ticked) {
    ticked = false;
    address = base_address;
    for (uint32_t i = in_0; i < in_count; ++i) {
      uint32_t bit = digitalRead(in_pins[i]);
      address += bit << i;
    }
#ifdef USE_PCA9624
    writeSeg7(hex2seg7[address]);
#endif
  }

  for (uint32_t i = out_0; i < out_count; ++i) {
    uint32_t bit = rom[address] >> i & 1;
    digitalWrite(out_pins[i], bit);
  }
#ifdef USE_PCA9624
  delay(50);
  writeSegDP(false);        // Light off decimal point
#endif
}
