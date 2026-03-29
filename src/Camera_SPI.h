/**
 * Camera_SPI.h
 * SPI plumbing for an OV2640 module in SPI capture mode (e.g. ArduCAM-style
 * breakout or similar). Real competitions often use vendor init blobs; here we
 * expose register pokes + a small FIFO read path for "basic vision" features.
 *
 * Hook up: VSYNC/HREF/RESET/PWDN per your board; SPI for data + SCCB/I2C for
 * sensor config on some boards — this header focuses on SPI clocked transfers.
 */
#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include <Arduino.h>
#include <SPI.h>

// OV2640 SPI-ish command bytes (common ArduCAM-style framing)
static const uint8_t kSpiWrite = 0x80;
static const uint8_t kSpiRead = 0x00;

class CameraSPI {
 public:
  CameraSPI(uint8_t csPin, uint8_t mosi = 11, uint8_t miso = 13, uint8_t sck = 12,
            uint32_t spiHz = 8000000)
      : _cs(csPin), _mosi(mosi), _miso(miso), _sck(sck), _hz(spiHz) {}

  /** Call SPI.begin(12, 13, 11, 10) in setup() before this (SCK, MISO, MOSI, CS). */
  void begin() {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    _settings = SPISettings(_hz, MSBFIRST, SPI_MODE0);
    Serial.printf("[Camera_SPI] SCK=%u MISO=%u MOSI=%u CS=%u @ %u Hz (SPI.begin done in setup)\n",
                  _sck, _miso, _mosi, _cs, (unsigned)_hz);
  }

  /** low-level: write sensor register (bank select handled by caller) */
  bool writeReg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    SPI.transfer(reg | kSpiWrite);
    SPI.transfer(val);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    delayMicroseconds(100);
    return true;
  }

  bool readReg(uint8_t reg, uint8_t &val) {
    SPI.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    SPI.transfer(reg & 0x7F);
    val = SPI.transfer(0x00);
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return true;
  }

  /**
   * Pull N bytes from the camera FIFO after a capture trigger (implementation
   * depends on board — stub fills with zeros if hardware not ready).
   * For pathfinding we often just need a downscaled row or histogram.
   */
  size_t readVisionChunk(uint8_t *buf, size_t maxLen) {
    if (!buf || maxLen == 0) return 0;
    SPI.beginTransaction(_settings);
    digitalWrite(_cs, LOW);
    // placeholder burst read pattern — replace with your module's READ_FIFO cmd
    for (size_t i = 0; i < maxLen; i++) {
      buf[i] = SPI.transfer(0x00);
    }
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    return maxLen;
  }

  /** super naive "path" metric: average brightness of first chunk (0..255) */
  uint8_t estimatePathBrightness(uint8_t *buf, size_t len) {
    if (!len) return 0;
    uint32_t s = 0;
    for (size_t i = 0; i < len; i++) s += buf[i];
    return (uint8_t)(s / len);
  }

 private:
  uint8_t _cs, _mosi, _miso, _sck;
  uint32_t _hz;
  SPISettings _settings;
};

#endif
