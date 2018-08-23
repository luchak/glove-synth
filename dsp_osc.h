#ifndef __DSP_OSC_H__
#define __DSP_OSC_H__

#include "audio.h"
#include "dsp_accumulator.h"

#define AUDIO_BITS 15
#define SR_SHIFT 6

// 24kHz sample rate, 22 bit phase
#define PHASE_BITS 22
extern const int32_t midi_note_phase_inc[128];

// Q22
extern const int32_t midi_note_freq_inv[128];

#define PHASE_MAX (1 << PHASE_BITS)
#define PHASE_HALF (1 << (PHASE_BITS - 1))

// int16_t sample_8_interp(const uint8_t* table, uint16_t phase) {
//   uint16_t phase_int = phase >> 8;
//   uint16_t phase_frac = phase & 0xFF;
// 
//   uint16_t sA = table[phase_int] * (0x100 - phase_frac);
//   uint16_t sB = table[(phase_int + 1) & 0xFF] * phase_frac;
//   int32_t value = sA + sB;
//   value -= 0x8000;
//   return (int16_t)value;
// }

class SquareOsc {
 public:
  SquareOsc(uint8_t note) : _phase(0) { setNote(note); }

  void setNote(uint8_t note) { _phase_inc = midi_note_phase_inc[note]; }

  q15_t tick() {
    _phase += _phase_inc;
    return _phase >= PHASE_HALF ? 0x7FFF : -0x8000;
  }

  void reset() { _phase = 0; }

 private:
  uint16_t _phase;
  uint16_t _phase_inc;
};

class SawOsc {
 public:
  SawOsc(uint8_t note) : _phase(0) { setNote(note); }

  void setNote(uint8_t note) {
    _phase_inc = midi_note_phase_inc[note];
    _freq_inv = midi_note_freq_inv[note];
  }

  q15_t tick() {
    int32_t val = (2 * _phase - PHASE_MAX) >> (PHASE_BITS - AUDIO_BITS);

    if (_phase < _phase_inc) {
      int32_t blep_phase = (((_phase * _freq_inv) >> PHASE_BITS) * (SAMPLE_RATE >> SR_SHIFT)) >>
                           (PHASE_BITS - (AUDIO_BITS + SR_SHIFT));
      val -= (2 * blep_phase - ((blep_phase * blep_phase) >> AUDIO_BITS) - (1 << AUDIO_BITS));
    } else if (_phase > (PHASE_MAX - _phase_inc)) {
      int32_t blep_phase = _phase - PHASE_MAX;
      blep_phase = (((blep_phase * _freq_inv) >> PHASE_BITS) * (SAMPLE_RATE >> SR_SHIFT)) >>
                   (PHASE_BITS - (AUDIO_BITS + SR_SHIFT));
      val -= (2 * blep_phase + ((blep_phase * blep_phase) >> AUDIO_BITS) + (1 << AUDIO_BITS));
    }

    _phase += _phase_inc;
    if (_phase >= PHASE_MAX) {
      _phase -= PHASE_MAX;
    }

    return val;
  }

  void reset() { _phase = 0; }

 private:
  int32_t _phase;
  int32_t _phase_inc;
  int32_t _freq_inv;
};

#undef PHASE_BITS
#undef PHASE_HALF
#undef PHASE_MAX
#undef AUDIO_BITS
#undef SR_SHIFT

#endif __DSP_OSC_H__
