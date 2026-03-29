/**
 * MotorDriver_PWM.h
 * Simple L298N wrapper for ESP32 LEDC PWM on enable pins.
 * IN pins are digital; ENA/ENB use LEDC only (never analogWrite()).
 *
 * Core 2.x: ledcSetup + ledcAttachPin + ledcWrite(channel).
 * Core 3.x: ledcAttach(pin) + ledcWrite(pin) — picked automatically via esp_arduino_version.h.
 *
 * ESP32-S3: GPIO39–44 are valid outputs; ENA/ENB get normal LEDC PWM (no dedicated
 * “motor” pins). Note: GPIO39–42 are the chip’s default JTAG (MTCK/MTDO/MTDI/MTMS);
 * with motors on those nets, turn off JTAG in firmware or your debugger may fight
 * the L298N. IN1/IN2 on 43–44 are outside that block.
 *
 * NOTE: Duty is scaled assuming a ~9V nominal battery pack. Fresh cells read
 * higher (~9.6V) but under load the pack sags — we cap effective PWM so the
 * motors don't brown out the ESP32 rail when both sides spike. Tune MAX_DUTY_PCT.
 */
#ifndef MOTOR_DRIVER_PWM_H
#define MOTOR_DRIVER_PWM_H

#include <Arduino.h>

#ifndef __has_include
#define __has_include(x) 0
#endif
#if __has_include("esp_arduino_version.h")
#include "esp_arduino_version.h"
#endif

// Arduino-ESP32 3.0+ uses pin-based LEDC; 2.x uses channel + ledcAttachPin.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
#define MOTOR_LEDC_USE_PIN_API 1
#else
#define MOTOR_LEDC_USE_PIN_API 0
#endif

// 9V pack: leave headroom so simultaneous accel doesn't dip logic supply too hard
#ifndef MOTOR_MAX_DUTY_PCT
#define MOTOR_MAX_DUTY_PCT 85  // percent of LEDC resolution range
#endif

class MotorDriverPWM {
 public:
  MotorDriverPWM(uint8_t pinEnA, uint8_t pinEnB, uint8_t pinIn1, uint8_t pinIn2,
                 uint8_t pinIn3, uint8_t pinIn4, uint32_t pwmFreqHz = 5000,
                 uint8_t pwmBits = 8)
      : _ena(pinEnA),
        _enb(pinEnB),
        _in1(pinIn1),
        _in2(pinIn2),
        _in3(pinIn3),
        _in4(pinIn4),
        _freq(pwmFreqHz),
        _bits(pwmBits),
        _chA(0),
        _chB(1) {}

  void begin() {
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    pinMode(_in3, OUTPUT);
    pinMode(_in4, OUTPUT);

    pwmAttachEnables();

#if MOTOR_LEDC_USE_PIN_API
    Serial.println("[MotorDriver] L298N PWM: core 3.x (ledcAttach + ledcWrite pin)");
#else
    Serial.println("[MotorDriver] L298N PWM: core 2.x (ledcSetup + ledcAttachPin + channel)");
#endif

    stop();
  }

  /** speed: -255..255-ish, sign = direction */
  void setLeftMotor(int16_t speed) {
    bool fwd = speed >= 0;
    uint16_t mag = (uint16_t)abs(speed);
    mag = scaleDuty(mag);
    digitalWrite(_in1, fwd ? HIGH : LOW);
    digitalWrite(_in2, fwd ? LOW : HIGH);
    pwmWriteEna(mag);
  }

  void setRightMotor(int16_t speed) {
    bool fwd = speed >= 0;
    uint16_t mag = (uint16_t)abs(speed);
    mag = scaleDuty(mag);
    digitalWrite(_in3, fwd ? HIGH : LOW);
    digitalWrite(_in4, fwd ? LOW : HIGH);
    pwmWriteEnb(mag);
  }

  void setTankDrive(int16_t left, int16_t right) {
    setLeftMotor(left);
    setRightMotor(right);
  }

  void stop() {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    digitalWrite(_in3, LOW);
    digitalWrite(_in4, LOW);
    pwmWriteEna(0);
    pwmWriteEnb(0);
  }

 private:
  uint8_t _ena, _enb, _in1, _in2, _in3, _in4;
  uint32_t _freq;
  uint8_t _bits;
  uint8_t _chA, _chB;  // legacy core: LEDC channel indices for ENA/ENB

  void pwmAttachEnables() {
#if MOTOR_LEDC_USE_PIN_API
    if (!ledcAttach(_ena, (double)_freq, _bits)) {
      Serial.println("[MotorDriver] ledcAttach ENA failed");
    }
    if (!ledcAttach(_enb, (double)_freq, _bits)) {
      Serial.println("[MotorDriver] ledcAttach ENB failed");
    }
#else
    ledcSetup(_chA, _freq, _bits);
    ledcSetup(_chB, _freq, _bits);
    ledcAttachPin(_ena, _chA);
    ledcAttachPin(_enb, _chB);
#endif
  }

  void pwmWriteEna(uint16_t duty) {
#if MOTOR_LEDC_USE_PIN_API
    ledcWrite(_ena, duty);
#else
    ledcWrite(_chA, duty);
#endif
  }

  void pwmWriteEnb(uint16_t duty) {
#if MOTOR_LEDC_USE_PIN_API
    ledcWrite(_enb, duty);
#else
    ledcWrite(_chB, duty);
#endif
  }

  uint16_t scaleDuty(uint16_t raw) {
    uint32_t maxVal = (1u << _bits) - 1u;
    uint32_t cap = (maxVal * (uint32_t)MOTOR_MAX_DUTY_PCT) / 100u;
    if (raw > maxVal) raw = (uint16_t)maxVal;
    uint32_t out = (raw * cap) / maxVal;
    return (uint16_t)out;
  }
};

#endif
