#include "control.h"

#include <Arduino.h>
#include <Wire.h>

uint8_t touch_active;
int16_t accel_sensor[3];

#define CONTROL_IRQ_PRIORITY 3

template <int16_t c0,
          uint8_t s0,
          int16_t c1,
          uint8_t s1,
          int16_t cgain,
          uint8_t sgain_pre,
          uint8_t sgain_post>
class ButterworthFilter {
 public:
  ButterworthFilter() {
    memset(xv_, 0, sizeof(xv_));
    memset(yv_, 0, sizeof(yv_));
  }

  int16_t tick(int32_t in) {
    in >>= sgain_pre;
    int32_t out = (xv_[0] + in) + (2 * xv_[1]) + ((c0 * yv_[0]) >> s0) + ((c1 * yv_[1]) >> s1);
    xv_[0] = xv_[1];
    xv_[1] = in;
    yv_[0] = yv_[1];
    yv_[1] = out;
    return (out * cgain) >> sgain_post;
  }

 private:
  int32_t xv_[2];
  int32_t yv_[2];
};

typedef ButterworthFilter<-980, 10, 2003, 10, 1, 7, 5> Butterworth5Hz;
typedef ButterworthFilter<-937, 10, 1957, 10, 31, 6, 9> Butterworth10Hz;

class Accelerometer {
 public:
  Accelerometer(uint8_t addr) : addr_(addr), i2c_(&Wire1) {}

  void begin() {
    i2c_->begin();

    Serial.println("device ID?");
    uint8_t deviceid = get8_(0x0F);
    Serial.print("device ID: ");
    Serial.print(deviceid);
    Serial.println();

    // Set 400 Hz data rate (shifted) and enable all 3 axes
    set8_(0x20, (0x07 << 4) | 0x07);
  }

  void read() {
    i2c_->beginTransmission(addr_);
    i2c_->write(0x28 | 0x80);  // 0x80 for autoincrement
    i2c_->endTransmission();
    i2c_->requestFrom(addr_, 6);

    x_ = i2c_->read();
    x_ |= ((uint16_t)i2c_->read()) << 8;
    y_ = i2c_->read();
    y_ |= ((uint16_t)i2c_->read()) << 8;
    z_ = i2c_->read();
    z_ |= ((uint16_t)i2c_->read()) << 8;
  }

  inline int16_t x() const { return x_; }
  inline int16_t y() const { return y_; }
  inline int16_t z() const { return z_; }

 private:
  void set8_(uint8_t reg, uint8_t value) {
    i2c_->beginTransmission((uint8_t)addr_);
    i2c_->write((uint8_t)reg);
    i2c_->write((uint8_t)value);
    i2c_->endTransmission();
  }

  uint8_t get8_(uint8_t reg) {
    i2c_->beginTransmission(addr_);
    i2c_->write((uint8_t)reg);
    i2c_->endTransmission();

    i2c_->requestFrom(addr_, 1);
    return i2c_->read();
  }

  uint8_t addr_;
  TwoWire* i2c_;

  int16_t x_;
  int16_t y_;
  int16_t z_;
};

Accelerometer accel(0x19);

static control_callback_t control_callback;

void control_init(control_callback_t callback) {
  control_callback = callback;

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;  // reset
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;  // reset
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  // Set Timer counter Mode to 16 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  // set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
  // set TC5 timer counter based off of the system clock and the user defined sample rate or
  // waveform
  TC5->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / CONTROL_RATE - 1);
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
    ;

  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, CONTROL_IRQ_PRIORITY);
  NVIC_EnableIRQ(TC5_IRQn);

  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
  pinMode(A7, INPUT_PULLUP);

  touch_active = 0;
}

void control_begin() {
  accel.begin();

  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
    ;
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
    ;
}

Butterworth10Hz xfilt;
Butterworth10Hz yfilt;
Butterworth10Hz zfilt;

void TC5_Handler() {
  // touch_update();
  uint8_t new_touch_active = 0;
  if (digitalRead(A1) == LOW)
    new_touch_active |= (1 << 0);
  if (digitalRead(A2) == LOW)
    new_touch_active |= (1 << 1);
  if (digitalRead(A3) == LOW)
    new_touch_active |= (1 << 2);
  if (digitalRead(A4) == LOW)
    new_touch_active |= (1 << 3);
  if (digitalRead(A5) == LOW)
    new_touch_active |= (1 << 4);
  if (digitalRead(A6) == LOW)
    new_touch_active |= (1 << 5);
  if (digitalRead(A7) == LOW)
    new_touch_active |= (1 << 6);
  touch_active = new_touch_active;

  control_callback();
  accel.read();

  accel_sensor[0] = xfilt.tick(accel.x());
  accel_sensor[1] = yfilt.tick(accel.y());
  accel_sensor[2] = zfilt.tick(accel.z());
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}
