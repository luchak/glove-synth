#ifndef __NEOPIXEL_H__
#define __NEOPIXEL_H__

#include <cstdint>

#define NEOPIXEL_NUM_LEDS 10

// Uses:
// - SERCOM5
// - DMA channel 1
// - PA22?, PA23?, PB22?, PB23

void neopixel_init();
void neopixel_begin();
void neopixel_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b);
void neopixel_clear(uint8_t idx);

#endif  // __NEOPIXEL_H__
