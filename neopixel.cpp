#include "neopixel.h"

#include <Arduino.h>

#include "dma.h"

#define NEOPIXEL_RESET_BUFFER_LENGTH 90

static uint8_t neopixel_data[NEOPIXEL_NUM_LEDS * 3 * 3 + NEOPIXEL_RESET_BUFFER_LENGTH];

void neopixel_init() {
  // put your setup code here, to run once:
  PM->APBCMASK.bit.SERCOM5_ = 1;
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_ID_SERCOM5_CORE;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;
  //    const SERCOM_SPI_CTRLA_Type ctrla = {
  //      .bit.DORD = 0, // MSB first
  //      .bit.CPHA = 0, // Mode 0
  //      .bit.CPOL = 0,
  //      .bit.FORM = 0, // SPI frame
  //      .bit.DIPO = 0, // MISO on PAD[0]
  //      .bit.DOPO = 2, // MOSI on PAD[3], SCK on PAD[1], SS_ on PAD[2]
  //      .bit.MODE = 3  // Master
  //    };
  //    SERCOM5->SPI.CTRLA.reg = ctrla.reg;
  //    const SERCOM_SPI_CTRLB_Type ctrlb = {
  //      .bit.RXEN = 0,   // RX disabled
  //      .bit.MSSEN = 1,  // HW SS
  //      .bit.CHSIZE = 0 // 8-bit
  //    };

  SERCOM5->SPI.CTRLA.bit.ENABLE = 0;
  while (SERCOM5->SPI.SYNCBUSY.bit.ENABLE)
    ;

  SERCOM_SPI_CTRLA_Type ctrla;
  ctrla.bit.DORD = 0;
  ctrla.bit.CPHA = 0;
  ctrla.bit.CPOL = 0;
  ctrla.bit.FORM = 0;
  ctrla.bit.DIPO = 0;
  ctrla.bit.DOPO = 2;
  ctrla.bit.MODE = 3;
  SERCOM5->SPI.CTRLA.reg = ctrla.reg;

  SERCOM_SPI_CTRLB_Type ctrlb;
  ctrlb.bit.RXEN = 0;
  ctrlb.bit.MSSEN = 0;
  ctrlb.bit.CHSIZE = 0;
  SERCOM5->SPI.CTRLB.reg = ctrlb.reg;

  SERCOM5->SPI.BAUD.reg = 9;  // Rate is clock / 2

  // Mux for SERCOM5 PA22,PA23,PB22,PB23
  //    PORT->Group[PORTA].PINCFG[22].bit.PMUXEN=1;
  //    PORT->Group[PORTA].PINCFG[23].bit.PMUXEN=1;
  //    PORT->Group[PORTA].PMUX[11].bit.PMUXE = 0x03;
  //    PORT->Group[PORTA].PMUX[11].bit.PMUXO = 0x03;
  // PORT->Group[PORTB].PINCFG[22].bit.PMUXEN=1;
  PORT->Group[PORTB].PINCFG[23].bit.PMUXEN = 1;
  // PORT->Group[PORTB].PMUX[11].bit.PMUXE = 0x03;
  PORT->Group[PORTB].PMUX[11].bit.PMUXO = 0x03;

  SERCOM5->SPI.CTRLA.bit.ENABLE = 1;
  while (SERCOM5->SPI.SYNCBUSY.bit.ENABLE)
    ;

  memset(neopixel_data, 0, sizeof(neopixel_data));
  for (int i = 0; i < NEOPIXEL_NUM_LEDS; i++) {
    neopixel_clear(i);
  }
  dma_init_neopixel(neopixel_data, sizeof(neopixel_data), SERCOM5_DMAC_ID_TX,
                    (uint32_t)&SERCOM5->SPI.DATA.reg);
}

void neopixel_begin() {
  dma_start_neopixel();
}

static void neopixel_set_channel(uint8_t idx, uint8_t channel, uint8_t value) {
  uint32_t encoded = 0;
  for (int i = 0; i < 8; i++) {
    if ((value >> i) & 0x1) {
      encoded |= 0b110 << (3 * i);
    } else {
      encoded |= 0b100 << (3 * i);
    }
  }

  uint8_t* data = neopixel_data + 3 * (3 * idx + channel);
  *data++ = (encoded >> 16) & 0xFF;
  *data++ = (encoded >> 8) & 0xFF;
  *data++ = (encoded)&0xFF;
}

void neopixel_set(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
  neopixel_set_channel(idx, 0, g);
  neopixel_set_channel(idx, 1, r);
  neopixel_set_channel(idx, 2, b);
}

void neopixel_clear(uint8_t idx) {
  neopixel_set(idx, 0, 0, 0);
}
