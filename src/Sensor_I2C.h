/**
 * Sensor_I2C.h
 * Thin I2C helpers for an MPU6050-class IMU (accel + gyro).
 * Used on the rover for heading-ish estimates and bump detection.
 * Nothing fancy — just reads raw registers.
 */
#ifndef SENSOR_I2C_H
#define SENSOR_I2C_H

#include <Arduino.h>
#include <Wire.h>

// MPU6050 defaults
static const uint8_t kMpuAddr = 0x68;
static const uint8_t kRegWhoAmI = 0x75;
static const uint8_t kRegPwrMgmt = 0x6B;
static const uint8_t kRegAccel = 0x3B;  // burst read from here
static const uint8_t kRegGyro = 0x43;

class SensorI2C {
 public:
  explicit SensorI2C(uint8_t sda = 15, uint8_t scl = 17, uint32_t clockHz = 400000)
      : _sda(sda), _scl(scl), _clock(clockHz), _addr(kMpuAddr) {}

  /** Call Wire.begin(15, 17) (or matching SDA/SCL) in setup() before this. */
  bool begin() {
    Serial.printf("[Sensor_I2C] SDA=%u SCL=%u (Wire already started)\n", _sda, _scl);
    Wire.setClock(_clock);
    delay(50);

    uint8_t who = 0;
    if (!readReg(kRegWhoAmI, &who, 1)) {
      Serial.println("[Sensor_I2C] WHO_AMI read failed — check wiring / addr");
      return false;
    }
    Serial.printf("[Sensor_I2C] WHO_AMI = 0x%02X (expect 0x68 for MPU6050)\n", who);

    // wake MPU6050 (clear sleep)
    if (!writeReg(kRegPwrMgmt, 0x00)) {
      Serial.println("[Sensor_I2C] PWR_MGMT write failed");
      return false;
    }
    delay(10);
    return true;
  }

  bool readAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t buf[6];
    if (!readReg(kRegAccel, buf, 6)) return false;
    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    return true;
  }

  bool readGyroRaw(int16_t &gx, int16_t &gy, int16_t &gz) {
    uint8_t buf[6];
    if (!readReg(kRegGyro, buf, 6)) return false;
    gx = (int16_t)((buf[0] << 8) | buf[1]);
    gy = (int16_t)((buf[2] << 8) | buf[3]);
    gz = (int16_t)((buf[4] << 8) | buf[5]);
    return true;
  }

  /** crude "turn rate" for path corrections — uses Z gyro only */
  float gyroZdps() {
    int16_t gx, gy, gz;
    if (!readGyroRaw(gx, gy, gz)) return 0.0f;
    // default ±250 dps -> 131 LSB/dps
    return gz / 131.0f;
  }

 private:
  uint8_t _sda, _scl;
  uint32_t _clock;
  uint8_t _addr;

  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
  }

  bool readReg(uint8_t reg, uint8_t *dst, size_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t got = Wire.requestFrom((int)_addr, (int)len);
    if (got != len) return false;
    for (size_t i = 0; i < len; i++) dst[i] = Wire.read();
    return true;
  }
};

#endif
