#ifndef __DSP_ACCUMULATOR_H__
#define __DSP_ACCUMULATOR_H__

#include <Arduino.h>

typedef int16_t q15_t;
const q15_t ONE_HALF = 1 << 14;
const uint8_t Q = 15;

class Accumulator {
 public:
  Accumulator() : val_(0) {}
  Accumulator(q15_t init) : val_(init) {}
  Accumulator(const Accumulator& other) : val_(other.val_) {}

  inline void set(q15_t x) { val_ = x; }

  inline void set(const Accumulator& other) { val_ = other.val_; }

  inline void add(const Accumulator& other) { val_ += other.val_; }

  inline void add(q15_t x) { val_ += x; }

  inline void sub(const Accumulator& other) { val_ -= other.val_; }

  inline void sub(q15_t x) { val_ -= x; }

  inline void madd(const Accumulator& other, const Accumulator& scale) {
    val_ += (other.val_ * scale.val_) >> Q;
  }

  inline void madd(const Accumulator& other, q15_t scale) {
    val_ += (other.val_ * (int32_t)scale) >> Q;
  }

  inline void madd(q15_t x, q15_t scale) { val_ += ((int32_t)x * (int32_t)scale) >> Q; }

  inline void madd_int_shift(q15_t x, q15_t scale, uint16_t post_shift = 0) {
    val_ += ((int32_t)x * (int32_t)scale) >> post_shift;
  }

  inline void mul(q15_t x) { val_ = (val_ * (int32_t)x) >> Q; }

  inline void mul_int(int16_t x) { val_ *= (int32_t)x; }

  inline void mul_int_shift(int16_t x, uint16_t post_shift) {
    val_ = (val_ * (int32_t)x) >> post_shift;
  }

  inline void asr_floor(uint32_t shift) { val_ >>= shift; }

  inline void asr_round(uint32_t shift) {
    val_ += 1 << (shift - 1);
    val_ >>= shift;
  }

  inline void asr_zero(uint32_t shift) {
    int32_t sign = (val_ >> 31) & 1;
    val_ = (val_ >> shift) + sign;
  }

  inline q15_t wrap_clip() const { return (q15_t)val_; }

  inline q15_t hard_clip() const { return min(max(val_, -0x8000), 0x7FFF); }

  inline q15_t soft_clip() const {
    const uint8_t QSAFE = 13;
    int32_t x = val_ >> (Q - QSAFE);
    x = x > (3 << 13) ? (3 << 13) : x;
    x = x < (-(3 << 13)) ? (-(3 << 13)) : x;
    int32_t x2 = (x * x) >> QSAFE;
    int32_t num = (27 << QSAFE) + x2;
    int32_t den = (27 << QSAFE) + 9 * x2;
    int32_t frac = (num * (1 << QSAFE)) / den;
    int32_t result = (frac * x) >> (2 * QSAFE - Q);
    return min(max(result, -0x8000), 0x7FFF);
  }

 private:
  int32_t val_;
};

#endif  // __DSP_ACCUMULATOR_H__
