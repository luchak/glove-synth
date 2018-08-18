#ifndef __DSP_DELAY_H__
#define __DSP_DELAY_H__

#include "dsp_accumulator.h"

#define DSP_DELAY_ULAW_BIAS 0x84 /* define the add-in bias for 16 bit samples */
#define DSP_DELAY_ULAW_CLIP 32635

inline uint8_t linear2ulaw(q15_t sample) {
  static uint8_t exp_lut[256] = {
      0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
      4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
      5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
      6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
      6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
      7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
      7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
      7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
      7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7};

  /* Get the sample into sign-magnitude. */
  uint8_t sign = (sample >> 8) & 0x80; /* set aside the sign */
  if (sign != 0)
    sample = -sample; /* get magnitude */
  if (sample > DSP_DELAY_ULAW_CLIP)
    sample = DSP_DELAY_ULAW_CLIP; /* clip the magnitude */

  /* Convert from 16 bit linear to ulaw. */
  sample = sample + DSP_DELAY_ULAW_BIAS;
  uint8_t exponent = exp_lut[(sample >> 7) & 0xFF];
  uint8_t mantissa = (sample >> (exponent + 3)) & 0x0F;
  uint8_t ulawbyte = ~(sign | (exponent << 4) | mantissa);

  return ulawbyte;
}

inline q15_t ulaw2linear(uint8_t ulawbyte) {
  static q15_t exp_lut[8] = {0, 132, 396, 924, 1980, 4092, 8316, 16764};

  ulawbyte = ~ulawbyte;
  uint8_t sign = (ulawbyte & 0x80);
  uint8_t exponent = (ulawbyte >> 4) & 0x07;
  q15_t mantissa = ulawbyte & 0x0F;
  q15_t sample = exp_lut[exponent] + (mantissa << (exponent + 3));
  if (sign != 0)
    sample = -sample;

  return sample;
}

class DelayLine {
 public:
  DelayLine(uint16_t delay_size) : pos_(0), size_(delay_size) {
    buffer_ = new q15_t[size_];
    memset(buffer_, 0, size_ * sizeof(q15_t));
  }
  ~DelayLine() { delete[] buffer_; }
  inline q15_t read() const { return buffer_[pos_]; }
  inline void write(q15_t x) {
    buffer_[pos_] = x;
    pos_ += 1;
    if (pos_ >= size_) {
      pos_ -= size_;
    }
  }

 private:
  uint16_t pos_;
  uint16_t size_;
  q15_t* buffer_;
};

class DelayLineULaw {
 public:
  DelayLineULaw(uint16_t delay_size) : pos_(0), size_(delay_size) {
    buffer_ = new uint8_t[size_];
    for (uint16_t i = 0; i < delay_size; i++) {
      buffer_[i] = 0xFF;
    }
  }
  ~DelayLineULaw() { delete[] buffer_; }
  inline q15_t read() const { return ulaw2linear(buffer_[pos_]); }
  inline void write(q15_t x) {
    buffer_[pos_] = linear2ulaw(x);
    pos_ += 1;
    if (pos_ >= size_) {
      pos_ -= size_;
    }
  }

 private:
  uint16_t pos_;
  uint16_t size_;
  uint8_t* buffer_;
};

template <uint8_t bits,
          uint16_t len,
          uint16_t bufsize = (1 << bits),
          uint16_t mask = (bufsize - 1),
          uint16_t lead = (bufsize - len)>
class DelayLineFastULaw {
 public:
  DelayLineFastULaw() : pos_(0) { memset(buffer_, 0xFF, bufsize); }
  ~DelayLineFastULaw() {}

  inline q15_t read(uint8_t offset) const {
    return ulaw2linear(buffer_[(pos_ + lead + offset) & mask]);
  }

  inline void write(q15_t x) {
    buffer_[pos_] = linear2ulaw(x);
    pos_ = (pos_ + 1) & mask;
  }

  void info() {
    Serial.println("DELAY PARAMS");
    Serial.println(bits);
    Serial.println(len);
    Serial.println(bufsize);
    Serial.println(mask);
    Serial.println(lead);
    Serial.println(pos_);
    Serial.println((pos_ + bufsize - (lead + 0)) & mask);
    Serial.println((uint32_t)buffer_);
    Serial.println("------");
  }

 private:
  uint32_t pos_;
  uint8_t buffer_[bufsize];
};

class ReverbFDN4 {
 public:
  ReverbFDN4() : delay0_(), delay1_(), delay2_(), delay3_() {}

  ~ReverbFDN4() {}

  inline q15_t tick(q15_t in) {
    int32_t delay_in0 = delay0_.read(0);
    int32_t delay_in1 = delay1_.read(0);
    int32_t delay_in2 = delay2_.read(0);
    int32_t delay_in3 = delay3_.read(0);

    int32_t sum = delay_in0 + delay_in1 + delay_in2 + delay_in3;
    int32_t halfsum = sum >> 1;
    q15_t qin = in >> 2;

    delay0_.write(((delay_in0 - halfsum) * 3 >> 2) + qin);
    delay1_.write(((delay_in1 - halfsum) * 3 >> 2) + qin);
    delay2_.write(((delay_in2 - halfsum) * 3 >> 2) + qin);
    delay3_.write(((delay_in3 - halfsum) * 7 >> 3) + qin);

    return sum;
  }

  void info() {
    delay0_.info();
    delay1_.info();
    delay2_.info();
    delay3_.info();
  }

 private:
  DelayLineFastULaw<12, 3181> delay0_;
  DelayLineFastULaw<12, 4093> delay1_;
  DelayLineFastULaw<12, 3617> delay2_;
  DelayLineFastULaw<13, 8191> delay3_;
  //    DelayLineFastULaw<11, 1301> delay0_;
  //    DelayLineFastULaw<11, 1571> delay1_;
  //    DelayLineFastULaw<11, 1811> delay2_;
  //    DelayLineFastULaw<11, 2039> delay3_;
};

#endif  // __DSP_DELAY_H__
