#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <cstdint>

#define CONTROL_RATE 48000

#define TOUCH_NUM_PADS 7

// Uses:
// - GCLK4 (REQUIRES audio to have set it up)
// - GCLK3 (via touch lib)
// - TC5

extern uint8_t touch_active;
extern int16_t accel_sensor[3];

typedef void (*control_callback_t)();

// YOU MUST CALL audio_init() FIRST TO SET UP THE CLOCK
void control_init(control_callback_t callback);

void control_begin();

#endif  // __CONTROL_H__
