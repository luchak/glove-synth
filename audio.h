#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <cstdint>

#include "dma.h"

#define SAMPLE_RATE 24000

// Uses:
// - GCLK4
// - TC4
// - DMA channel 0

typedef void (*audio_callback_t)(uint16_t* buf, uint16_t len);

void audio_init(audio_callback_t audio_callback);
void audio_begin();

#endif  // __AUDIO_H__
