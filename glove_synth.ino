#include <Arduino.h>

#include "audio.h"
#include "control.h"
#include "dma.h"
#include "neopixel.h"

#include "dsp_accumulator.h"
#include "dsp_delay.h"
#include "dsp_filter.h"
#include "dsp_osc.h"

#define DO10X(code) \
  code;             \
  code;             \
  code;             \
  code;             \
  code;             \
  code;             \
  code;             \
  code;             \
  code;             \
  code

SawOsc saw_oscs[] = {SawOsc(57), SawOsc(58), SawOsc(59), SawOsc(60), SawOsc(61),
                     SawOsc(62), SawOsc(63), SawOsc(64), SawOsc(65), SawOsc(66),
                     SawOsc(67), SawOsc(68), SawOsc(69)};
SquareOsc square_oscs[] = {SquareOsc(57), SquareOsc(58), SquareOsc(59), SquareOsc(60),
                           SquareOsc(61), SquareOsc(62), SquareOsc(63), SquareOsc(64),
                           SquareOsc(65), SquareOsc(66), SquareOsc(67), SquareOsc(68),
                           SquareOsc(69)};
uint16_t voice_mask = 0;

DelayLineULaw delay_line(4000);
// ReverbFDN4 reverb;
SVF svf(1000);
int16_t freq_inc = 1;

uint32_t numticks = 0;
uint32_t audio_cb_micros = 0;

void audio_callback(uint16_t* buf, uint16_t len) {
  //    uint32_t next_audio_cb_micros = micros();
  //    Serial.println(next_audio_cb_micros - audio_cb_micros);
  //    audio_cb_micros = next_audio_cb_micros;

  uint16_t svf_freq = svf.freq();
  int32_t param = ((int32_t)accel_sensor[1] + (1 << 15)) >> 1;
  int32_t param2 = (param * param) >> 15;
  int32_t param3 = (param2 * param) >> 15;
  svf_freq = 150 + ((param3 * (11500 - 150)) >> 15);
  // svf_freq += (((int32_t)accel_sensor[0] >> 4) * (int32_t)svf_freq) >> 18;
  // svf_freq = ((int32_t)accel_sensor[0] * (6000 - 400)) >> 15;
  svf_freq = svf_freq < 250 ? 250 : svf_freq;
  svf_freq = svf_freq > 7500 ? 7500 : svf_freq;
  svf.setFreq(svf_freq);

  for (int i = 0; i < len; i++) {
    numticks++;

    Accumulator mix(0);
    for (int i = 0; i < 16; i++) {
      if (voice_mask & (1 << i)) {
        q15_t saw = saw_oscs[i].tick();
        q15_t square = square_oscs[i].tick();
        int16_t osc_mix_frac = (accel_sensor[0] >> 1) + 0x8000;
        osc_mix_frac = 0x7EEE;
        int32_t osc_mix = osc_mix_frac * saw + (0x7EEE - osc_mix_frac) * square;
        mix.add(osc_mix >> 15);
      }
    }

    mix.asr_floor(3);

    //svf.tick(mix.hard_clip());
    //mix.set(svf.low());

    Accumulator feedback(delay_line.read());
    Accumulator dry_mix(mix);
    mix.add(feedback.soft_clip());
    feedback.mul_int(3);
    feedback.asr_round(2);
    feedback.add(dry_mix);
    if (numticks & 0x1)
      delay_line.write(feedback.soft_clip());

    // q15_t mix_reverb = reverb.tick(mix.soft_clip());
    // mix.add(mix_reverb >> 1);

    buf[i] = ((int32_t)mix.hard_clip() + 0x8000) >> 6;
  }
}

void control_callback() {
  uint16_t new_voice_mask = 0;
  //  if (touch_active[0]) new_voice_mask |= 0x0091;
  //  if (touch_active[1]) new_voice_mask |= 0x0224;
  //  if (touch_active[2]) new_voice_mask |= 0x0890;
  //  if (touch_active[3]) new_voice_mask |= 0x0221;
  //  if (touch_active[4]) new_voice_mask |= 0x0884;
  //  if (touch_active[5]) new_voice_mask |= 0x0211;
  //  if (touch_active[6]) new_voice_mask |= 0x0824;
  //  if (touch_active[0]) new_voice_mask |= 0x0081;
  //  if (touch_active[1]) new_voice_mask |= 0x0204;
  //  if (touch_active[2]) new_voice_mask |= 0x0810;
  //  if (touch_active[3]) new_voice_mask |= 0x1020;
  //  if (touch_active[4]) new_voice_mask |= 0x0084;
  //  if (touch_active[5]) new_voice_mask |= 0x0210;
  //  if (touch_active[6]) new_voice_mask |= 0x0820;
  //  if (touch_active & (1 << 0)) new_voice_mask |= 0x0001;
  //  if (touch_active & (1 << 1)) new_voice_mask |= 0x0004;
  //  if (touch_active & (1 << 2)) new_voice_mask |= 0x0010;
  //  if (touch_active & (1 << 3)) new_voice_mask |= 0x0020;
  //  if (touch_active & (1 << 4)) new_voice_mask |= 0x0080;
  //  if (touch_active & (1 << 5)) new_voice_mask |= 0x0200;
  //  if (touch_active & (1 << 6)) new_voice_mask |= 0x0800;
  if (touch_active & (1 << 0))
    new_voice_mask |= 0x0081;
  if (touch_active & (1 << 1))
    new_voice_mask |= 0x0204;
  if (touch_active & (1 << 2))
    new_voice_mask |= 0x0810;
  if (touch_active & (1 << 3))
    new_voice_mask |= 0x1020;
  if (touch_active & (1 << 4))
    new_voice_mask |= 0x0084;
  if (touch_active & (1 << 5))
    new_voice_mask |= 0x0210;
  if (touch_active & (1 << 6))
    new_voice_mask |= 0x0820;

  voice_mask = new_voice_mask;

  for (int i = 0; i < 7; i++) {
    if (touch_active & (1 << i)) {
      neopixel_set(9 - i, 0x08, 0x10, 0x00);
    } else {
      neopixel_clear(9 - i);
    }
  }
  // Serial.println(voice_mask);
}

uint32_t prof_start_time;
void prof_start() {
  prof_start_time = micros();
}
uint32_t prof_end() {
  uint32_t prof_end_time = micros();
  return prof_end_time - prof_start_time;
}

extern "C" char* sbrk(int i);

int FreeRam() {
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

#define REPS 100000
void setup() {
  //  Serial.begin(9600);
  //  while (!Serial);
  //
  //  reverb.info();
  //
  //  Accumulator test(1);
  //  q15_t scale = -0x8000;
  //  prof_start();
  //  for (int i = 0; i < REPS; i++) {
  //    DO10X(test.madd(1, 0x1));
  //  }
  //  uint32_t elapsed = prof_end();
  //  Serial.println(elapsed);
  //  Serial.println(test.hard_clip());

  // turn off speaker
  //  pinMode(11, OUTPUT);
  //  digitalWrite(11, LOW);

  analogReadResolution(10);
  analogWriteResolution(10);

  analogWrite(A0, 512);

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  Serial.print("Free RAM: ");
  Serial.println(FreeRam());

  // these init calls must be run in this exact order
  dma_init();
  audio_init(audio_callback);
  control_init(control_callback);
  neopixel_init();

  Serial.print("Free RAM: ");
  Serial.println(FreeRam());

  audio_begin();
  control_begin();
  neopixel_begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(analogRead(A1));
  //  lis3dh->read();
  //  accel_sensor[0] = lis3dh->x;
  //  accel_sensor[1] = lis3dh->y;
  //  accel_sensor[2] = lis3dh->z;
  //
  //  Serial.print(accel_sensor[0]);
  //  Serial.print(" ");
  //  Serial.print(accel_sensor[1]);
  //  Serial.print(" ");
  //  Serial.print(accel_sensor[2]);
  //  Serial.println("");
}
