#ifndef __DMA_H__
#define __DMA_H__

#include <cstdint>

#define DMA_NUM_CHANNELS 4

typedef uint16_t* (*dma_audio_callback_t)();

void dma_init();

void dma_init_audio(uint8_t buffer_len, uint32_t dma_trigger, dma_audio_callback_t callback);

void dma_start_audio();

void dma_init_neopixel(uint8_t* neopixel_buffer,
                       uint16_t neopixel_buffer_len,
                       uint32_t dma_trigger,
                       uint32_t dma_dst);

void dma_start_neopixel();

#endif  // __DMA_H__
