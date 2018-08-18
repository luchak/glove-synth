#ifndef __DSP_FILTER_H__
#define __DSP_FILTER_H__

#include "dsp_accumulator.h"

// Q24 ~= (2*pi*Fs) * SAMPLE_RATE, for SAMPLE_RATE = 24000
const int32_t svf_filter_freq_constant = 4390;

class SVF {
 public:
  SVF(int16_t freq) : q1_(192), low_(0), high_(0), band_(0) { setFreq(freq); }

  inline void setFreq(int16_t f) {
    freq_ = f;
    f >>= 1;
    f1_ = (int16_t)((svf_filter_freq_constant * (int32_t)f) >> (24 - 10));
  }

  inline void tick(q15_t x) {
    int32_t d1 = band_.hard_clip();
    low_.add((d1 * (int32_t)f1_) >> 10);
    high_.set(x);
    high_.sub(low_);
    high_.sub((d1 * (int32_t)q1_) >> 10);
    band_.add(((int32_t)(high_.hard_clip()) * (int32_t)f1_) >> 10);

    d1 = band_.hard_clip();
    low_.add((d1 * (int32_t)f1_) >> 10);
    high_.set(x);
    high_.sub(low_);
    high_.sub((d1 * (int32_t)q1_) >> 10);
    band_.add(((int32_t)(high_.hard_clip()) * (int32_t)f1_) >> 10);

    // notch_.set(high_);
    // notch_.add(low_);
  }

  inline int16_t freq() const { return freq_; }
  inline q15_t low() const { return low_.hard_clip(); }

 private:
  int16_t freq_;
  int16_t f1_;  // q10
  int16_t q1_;  // q10, range 2 to 0
  Accumulator low_;
  Accumulator high_;
  Accumulator band_;
};

#endif  // __DSP_FILTER_H__
