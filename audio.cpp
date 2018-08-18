#include "audio.h"

#include <Arduino.h>

#define BUFSIZE 8
#define DMA_TRIGGER TC4_DMAC_ID_OVF

static audio_callback_t audio_user_callback;
static uint16_t dma_buf[BUFSIZE * 2];
static uint16_t* front = dma_buf;
static uint16_t* back = dma_buf + BUFSIZE;

static uint16_t* handle_dma_callback() {
  audio_user_callback(back, BUFSIZE);

  uint16_t* tmp = front;
  front = back;
  back = tmp;

  return front;
}

void audio_init(audio_callback_t callback) {
  memset(dma_buf, 0, BUFSIZE * sizeof(uint16_t));
  memset(dma_buf + BUFSIZE, 0, BUFSIZE * sizeof(uint16_t));

  REG_GCLK_GENDIV =
      GCLK_GENDIV_DIV(1) |  // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
      GCLK_GENDIV_ID(4);    // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |        // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |  // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);         // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |      // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |  // Select GCLK4
                     GCLK_CLKCTRL_ID_TC4_TC5;  // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  // Normal (single slope) PWM operation: timers countinuously count up to PER
  // register value and then is reset to 0

  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;  // reset
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;  // reset
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  // Set Timer counter Mode to 16 bits
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  // set prescaler and enable TC5
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
  // set TC5 timer counter based off of the system clock and the user defined
  // sample rate or waveform
  TC4->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / SAMPLE_RATE - 1);
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;

  NVIC_DisableIRQ(TC4_IRQn);
  NVIC_ClearPendingIRQ(TC4_IRQn);
  // NVIC_SetPriority(TC4_IRQn, 0);
  // NVIC_EnableIRQ(TC4_IRQn);

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;

  audio_user_callback = callback;
  dma_init_audio(BUFSIZE, DMA_TRIGGER, handle_dma_callback);
}

void audio_begin() {
  dma_start_audio();
}
