#include "dma.h"

#include <Arduino.h>

#define DMA_AUDIO_CHANNEL 0
#define DMA_NEOPIXEL_CHANNEL 1
#define DMA_IRQ_PRIORITY 0

static __attribute__((__aligned__(16)))
DmacDescriptor dma_descriptors[DMA_NUM_CHANNELS] SECTION_DMAC_DESCRIPTOR;

static __attribute__((__aligned__(16)))
DmacDescriptor dma_writeback[DMA_NUM_CHANNELS] SECTION_DMAC_DESCRIPTOR;

static uint16_t* audio_next_buffer;
static dma_audio_callback_t audio_callback;
static uint8_t audio_buffer_len;

void dma_init() {
  // Set bus flags
  PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

  // Enable IRQ
  NVIC_DisableIRQ(DMAC_IRQn);
  NVIC_ClearPendingIRQ(DMAC_IRQn);
  NVIC_SetPriority(DMAC_IRQn, DMA_IRQ_PRIORITY);
  NVIC_EnableIRQ(DMAC_IRQn);

  // Set memory region
  DMAC->BASEADDR.reg = (uint32_t)dma_descriptors;
  DMAC->WRBADDR.reg = (uint32_t)dma_writeback;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);
}

void dma_init_audio(uint8_t buffer_len, uint32_t dma_trigger, dma_audio_callback_t callback) {
  audio_buffer_len = buffer_len;
  audio_callback = callback;

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_AUDIO_CHANNEL);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << DMA_AUDIO_CHANNEL));
  DMAC->CHCTRLB.reg =
      DMAC_CHCTRLB_LVL(0) | DMAC_CHCTRLB_TRIGSRC(dma_trigger) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;

  DmacDescriptor* descriptor = dma_descriptors + DMA_AUDIO_CHANNEL;
  descriptor->BTCTRL.bit.VALID = true;
  descriptor->BTCTRL.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE;  // No output events
  descriptor->BTCTRL.bit.BLOCKACT =
      DMAC_BTCTRL_BLOCKACT_INT;           // Fire an interrupt on block transfer complete
  descriptor->BTCTRL.bit.BEATSIZE = 0x1;  // DMAC_BTCTRL_BEATSIZE_HWORD;  // 16 bit
  descriptor->BTCTRL.bit.SRCINC = true;
  descriptor->BTCTRL.bit.DSTINC = false;
  descriptor->BTCTRL.bit.STEPSEL = true;
  descriptor->BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1;  // STEPSEL is set so n/a
  descriptor->BTCNT.reg = audio_buffer_len;
  descriptor->SRCADDR.reg = NULL;
  descriptor->DSTADDR.reg = (uint32_t)&DAC->DATA.reg;
  descriptor->DESCADDR.reg = NULL;
}

static void dma_audio_set_src(uint16_t* src) {
  DmacDescriptor* descriptor = dma_descriptors + DMA_AUDIO_CHANNEL;
  descriptor->SRCADDR.reg = (uint32_t)src + sizeof(uint16_t) * audio_buffer_len;
}

void dma_start_audio() {
  // start channel

  dma_audio_set_src(audio_callback());
  audio_next_buffer = audio_callback();

  DMAC->CHID.reg = DMAC_CHID_ID(DMA_AUDIO_CHANNEL);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void dma_init_neopixel(uint8_t* neopixel_buffer,
                       uint16_t neopixel_buffer_len,
                       uint32_t dma_trigger,
                       uint32_t dma_dst) {
  DMAC->CHID.reg = DMAC_CHID_ID(DMA_NEOPIXEL_CHANNEL);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1 << DMA_NEOPIXEL_CHANNEL));
  DMAC->CHCTRLB.reg = DMAC_CHCTRLB_TRIGSRC(dma_trigger) | DMAC_CHCTRLB_TRIGACT_BEAT;
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;

  DmacDescriptor* descriptor = dma_descriptors + DMA_NEOPIXEL_CHANNEL;
  descriptor->BTCTRL.bit.VALID = true;
  descriptor->BTCTRL.bit.EVOSEL = DMAC_BTCTRL_EVOSEL_DISABLE;  // No output events
  descriptor->BTCTRL.bit.BLOCKACT =
      DMAC_BTCTRL_BLOCKACT_NOACT;         // Fire an interrupt on block transfer complete
  descriptor->BTCTRL.bit.BEATSIZE = 0x0;  // DMAC_BTCTRL_BEATSIZE_HWORD;  // 16 bit
  descriptor->BTCTRL.bit.SRCINC = true;
  descriptor->BTCTRL.bit.DSTINC = false;
  descriptor->BTCTRL.bit.STEPSEL = 0;
  descriptor->BTCTRL.bit.STEPSIZE = DMAC_BTCTRL_STEPSIZE_X1;  // STEPSEL is set so n/a
  descriptor->BTCNT.reg = neopixel_buffer_len;
  descriptor->SRCADDR.reg = (uint32_t)(neopixel_buffer + neopixel_buffer_len - 1);
  descriptor->DSTADDR.reg = dma_dst;
  descriptor->DESCADDR.reg = (uint32_t)descriptor;
}

void dma_start_neopixel() {
  DMAC->CHID.reg = DMAC_CHID_ID(DMA_NEOPIXEL_CHANNEL);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void DMAC_Handler() {
  // interrupts DMAC_CHINTENCLR_TERR DMAC_CHINTENCLR_TCMPL DMAC_CHINTENCLR_SUSP
  // disable irqs ?
  __disable_irq();

  uint8_t active_channel = DMAC->INTPEND.bit.ID;  // get channel number
  DMAC->CHID.reg = DMAC_CHID_ID(active_channel);

  if (DMAC->CHINTFLAG.bit.TERR) {
    // error
    Serial.println("ERROR");
    digitalWrite(13, HIGH);
    DMAC->CHINTFLAG.reg &= DMAC_CHINTENCLR_TERR;
  }

  if (DMAC->CHINTFLAG.bit.TCMPL) {
    // transfer complete

    DMAC->CHINTFLAG.reg &= DMAC_CHINTENCLR_TCMPL;
    if (active_channel == DMA_AUDIO_CHANNEL) {
      dma_audio_set_src(audio_next_buffer);
      DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
      audio_next_buffer = audio_callback();
    }
  }

  DMAC->CHINTFLAG.reg &= DMAC_CHINTENCLR_SUSP;  // not handled; just clear flag

  __enable_irq();
}
