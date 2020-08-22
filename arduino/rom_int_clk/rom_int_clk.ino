/**
 * ROM emulator for TD4 using interrupt on CLK
 */
#include "td4_iface.h"
#include "td4_rom.h"

const uint32_t clock_pin = CLK;
const uint32_t base_address = 0;
const uint32_t default_address = 0;

volatile int ticked = false;

void tick() {
  ticked = true;
}

void setup() {
  for (uint32_t i = in_0; i < in_count; ++i) {
    pinMode(in_pins[i], INPUT);
  }
  for (uint32_t i = out_0; i < out_count; ++i) {
    pinMode(out_pins[i], OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(clock_pin), tick, RISING);
}

void loop() {
  static uint32_t address = default_address;

  noInterrupts();
  if (ticked) {
    ticked = false;
    address = base_address;
    for (uint32_t i = in_0; i < in_count; ++i) {
      uint32_t bit = digitalRead(in_pins[i]);
      address += bit << i;
    }
  }
  interrupts();

  for (uint32_t i = out_0; i < out_count; ++i) {
    uint32_t bit = rom[address] >> i & 1;
    digitalWrite(out_pins[i], bit);
  }
}
