#ifndef __td4_iface_h
#define __td4_iface_h

#include <stdint.h>

enum {
  RST = 2,
  CLK = 3,
};

enum { in_0 = 0, in_1, in_2, in_3, in_count };
const uint32_t in_pins[in_count] = { A0, A1, A2, A3 };

enum { out_0 = 0, out_1, out_2, out_3, out_4, out_5, out_6, out_7, out_count };
const uint32_t out_pins[out_count] = { 4, 5, 6, 7, 8, 9, 10, 11 };

#endif /* __td4_iface_h */
