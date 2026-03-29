/**
 * Camera_SPI.h
 * ArduCAM Mini 2MP (OV2640) over SPI + I2C (sensor) on the same Wire bus as MPU6050.
 *
 * - Bring-up: SPI self-test, OV2640 chip ID, BMP / RGB565 via ArduCAM::InitCAM().
 * - Resolution: ArduCAM BMP defaults to QVGA (320x240). After init we call
 *   OV2640_set_JPEG_size(OV2640_176x144) to apply the library's 176x144 timing
 *   block (same table as JPEG size selection — common for SPI Mini to shrink output).
 *   Real pixel layout is taken from the BMP header after each capture.
 * - FIFO: native ArduCAM sequence flush_fifo → clear_fifo_flag → start_capture →
 *   wait CAP_DONE → read_fifo_length → CS_LOW → set_fifo_burst → bulk read.
 * - Buffer: DMA-capable internal RAM (heap_caps). SPI bulk read uses ESP32
 *   SPI.transferBytes (driver-optimized; pairs with DMA-capable RX buffer).
 */
#ifndef CAMERA_SPI_H
#define CAMERA_SPI_H

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <esp_heap_caps.h>

// Project must supply memorysaver.h with #define OV2640_MINI_2MP (see include/)
#include <ArduCAM.h>

#ifndef OV2640_MINI_2MP
#error Define OV2640_MINI_2MP in include/memorysaver.h and use ArduCAM OV2640 Mini 2MP
#endif

// Worst-case BMP in FIFO: header + 320x240 RGB565 + margin
static const size_t kBmpFifoMaxBytes = 64u + (size_t)320u * 240u * 2u + 256u;

class CameraSPI {
 public:
  explicit CameraSPI(uint8_t csPin, uint32_t spiHz = 8000000u)
      : _cs(csPin), _hz(spiHz), _cam(nullptr), _frameDma(nullptr), _lastW(176), _lastH(144) {}

  ~CameraSPI() {
    if (_frameDma) {
      heap_caps_free(_frameDma);
      _frameDma = nullptr;
    }
    delete _cam;
    _cam = nullptr;
  }

  /**
   * Wire.begin(SDA,SCL) and SPI.begin(SCK,MISO,MOSI,CS) must run before this.
   * Sensor uses I2C @ 0x30 (ArduCAM) on the same Wire bus as MPU6050 @ 0x68 — no address clash.
   */
  bool begin() {
    _frameDma =
        (uint8_t *)heap_caps_aligned_alloc(16, kBmpFifoMaxBytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!_frameDma) {
      _frameDma = (uint8_t *)heap_caps_malloc(kBmpFifoMaxBytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    }
    if (!_frameDma) {
      Serial.println("[Camera_SPI] DMA framebuffer alloc failed");
      return false;
    }

    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
    SPI.setFrequency(_hz);

    _cam = new ArduCAM(OV2640, _cs);
    if (!_cam) {
      return false;
    }

    _cam->write_reg(ARDUCHIP_TEST1, 0x55);
    uint8_t t = _cam->read_reg(ARDUCHIP_TEST1);
    if (t != 0x55) {
      Serial.println("[Camera_SPI] ArduSPI test failed (check CS / SPI wiring)");
      return false;
    }

    uint8_t vid = 0, pid = 0;
    _cam->wrSensorReg8_8(0xff, 0x01);
    _cam->rdSensorReg8_8(0x0A, &vid);
    _cam->rdSensorReg8_8(0x0B, &pid);
    Serial.printf("[Camera_SPI] OV2640 PID 0x%02X%02X (expect 2642)\n", vid, pid);

    _cam->set_format(BMP);
    _cam->InitCAM();
    // Shrink output toward QCIF; same helper ArduCAM uses for JPEG size tables.
    _cam->OV2640_set_JPEG_size(OV2640_176x144);

    Serial.printf("[Camera_SPI] ArduCAM init OK, BMP target ~QCIF, DMA buf %u bytes\n",
                  (unsigned)kBmpFifoMaxBytes);
    return true;
  }

  uint16_t width() const { return _lastW; }
  uint16_t height() const { return _lastH; }

  uint8_t *frameBuffer() { return pixelDataPtr(); }
  const uint8_t *frameBuffer() const { return pixelDataPtr(); }

  size_t frameSizeBytes() const { return (size_t)_lastW * (size_t)_lastH * 2u; }

  /**
   * One BMP capture into DMA buffer; parses BMP header to find RGB565 payload and updates _lastW/H.
   */
  bool captureFrameQCIF_DMA() {
    if (!_cam || !_frameDma) return false;

    _cam->flush_fifo();
    _cam->clear_fifo_flag();
    _cam->start_capture();

    uint32_t t0 = millis();
    while (!_cam->get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
      if (millis() - t0 > 600u) {
        Serial.println("[Camera_SPI] capture timeout");
        return false;
      }
      yield();
    }

    uint32_t len = _cam->read_fifo_length();
    if (len == 0 || len > kBmpFifoMaxBytes) {
      Serial.printf("[Camera_SPI] bad FIFO len %lu\n", (unsigned long)len);
      return false;
    }

    _cam->CS_LOW();
    _cam->set_fifo_burst();
#if !(defined(ARDUCAM_SHIELD_V2) && defined(OV2640_CAM))
    SPI.transfer(0xFF);
#endif

    if (!spiReadIntoBuffer(_frameDma, len)) {
      _cam->CS_HIGH();
      return false;
    }
    _cam->CS_HIGH();

    if (!parseBmpRgb565(_frameDma, len)) {
      return false;
    }
    return true;
  }

 private:
  uint8_t _cs;
  uint32_t _hz;
  ArduCAM *_cam;
  uint8_t *_frameDma;
  uint16_t _lastW, _lastH;
  uint32_t _pixelOff = 0;

  uint8_t *pixelDataPtr() {
    if (!_frameDma) return nullptr;
    return _frameDma + _pixelOff;
  }

  /** ESP32 SPI: bulk read — keeps MOSI as 0xFF while clocking MISO into dst (DMA-friendly dst). */
  bool spiReadIntoBuffer(uint8_t *dst, size_t len) {
    static uint8_t tx[1024];
    const size_t chunk = sizeof(tx);
    size_t i = 0;
    while (i < len) {
      size_t n = len - i;
      if (n > chunk) n = chunk;
      memset(tx, 0xFF, n);
      SPI.transferBytes(tx, dst + i, n);
      i += n;
    }
    return true;
  }

  bool parseBmpRgb565(const uint8_t *buf, uint32_t len) {
    if (len < 54 || buf[0] != 'B' || buf[1] != 'M') {
      Serial.println("[Camera_SPI] FIFO not a BMP");
      _pixelOff = 0;
      return false;
    }
    uint32_t off = (uint32_t)buf[10] | ((uint32_t)buf[11] << 8) | ((uint32_t)buf[12] << 16) |
                   ((uint32_t)buf[13] << 24);
    uint32_t w = (uint32_t)buf[18] | ((uint32_t)buf[19] << 8) | ((uint32_t)buf[20] << 16) |
                 ((uint32_t)buf[21] << 24);
    uint32_t h = (uint32_t)buf[22] | ((uint32_t)buf[23] << 8) | ((uint32_t)buf[24] << 16) |
                 ((uint32_t)buf[25] << 24);
    if (h & 0x80000000u) h = (uint32_t)(-(int32_t)h);
    if (w == 0 || w > 400 || h == 0 || h > 400) {
      Serial.printf("[Camera_SPI] BMP dims bad: %lu x %lu\n", (unsigned long)w, (unsigned long)h);
      return false;
    }
    if (off + w * h * 2 > len) {
      Serial.println("[Camera_SPI] BMP pixel data past FIFO");
      return false;
    }
    _lastW = (uint16_t)w;
    _lastH = (uint16_t)h;
    _pixelOff = off;
    return true;
  }
};

#endif
