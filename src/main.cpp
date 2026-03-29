/**
 * ESP32 rover — competition firmware skeleton
 * Ties I2C IMU + SPI camera cues to differential drive.
 * Pathfinding here = reactive: keep floor/line brightness in view + damp yaw
 * with gyro. Replace gains with your tuning from the pit.
 */

#include <Arduino.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || \
    defined(ARDUINO_ESP32S3)
#include <driver/gpio.h>
#endif

#include <SPI.h>
#include <Wire.h>

#include "MotorDriver_PWM.h"
#include "Sensor_I2C.h"
#include "Camera_SPI.h"

// --- KiCad 2-layer PCB: ESP32-S3 pin map (must match hardware) ---
// L298N motor driver
static const uint8_t PIN_IN1 = 44;
static const uint8_t PIN_IN2 = 43;
static const uint8_t PIN_IN3 = 42;
static const uint8_t PIN_IN4 = 41;
static const uint8_t PIN_ENA = 40;
static const uint8_t PIN_ENB = 39;
// MPU6050 (Wire.begin(SDA, SCL))
static const uint8_t PIN_SDA = 15;
static const uint8_t PIN_SCL = 17;
// ArduCAM SPI: SPI.begin(SCK, MISO, MOSI, SS)
static const uint8_t PIN_SCK = 12;
static const uint8_t PIN_MISO = 13;
static const uint8_t PIN_MOSI = 11;
static const uint8_t PIN_CAM_CS = 10;

// timing
static const uint32_t SENSOR_MS = 20;
static const uint32_t CAMERA_MS = 100;
static const uint32_t PRINT_MS = 500;

SensorI2C imu(PIN_SDA, PIN_SCL);
MotorDriverPWM motors(PIN_ENA, PIN_ENB, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4);
CameraSPI cam(PIN_CAM_CS, PIN_MOSI, PIN_MISO, PIN_SCK);

uint32_t lastSens = 0;
uint32_t lastCam = 0;
uint32_t lastPrint = 0;

// control state (very simple)
float yawRateFiltered = 0.0f;
uint8_t lastBright = 128;

// PD-ish steering from brightness error + gyro damping
static int16_t computeLeft(int16_t base, float brightnessErr, float yawDps) {
  int32_t v = base + (int32_t)(brightnessErr * 0.8f) - (int32_t)(yawDps * 6.0f);
  if (v > 200) v = 200;
  if (v < -200) v = -200;
  return (int16_t)v;
}

static int16_t computeRight(int16_t base, float brightnessErr, float yawDps) {
  int32_t v = base - (int32_t)(brightnessErr * 0.8f) - (int32_t)(yawDps * 6.0f);
  if (v > 200) v = 200;
  if (v < -200) v = -200;
  return (int16_t)v;
}

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || \
    defined(ARDUINO_ESP32S3)
/** ESP32-S3: L298N uses GPIO39–42 (MTCK/TDO/TDI/TMS). Default USB-Serial-JTAG
 *  leaves these free as GPIO, but reset the pin mux before PWM/dir setup so
 *  nothing leaves them in a weird state. If OpenOCD loses sync later, you
 *  muxed external JTAG onto these pads via eFuse — don’t run this rover + GDB. */
static void roverPrepareL298nGpio(void) {
  gpio_reset_pin((gpio_num_t)PIN_ENB);
  gpio_reset_pin((gpio_num_t)PIN_ENA);
  gpio_reset_pin((gpio_num_t)PIN_IN4);
  gpio_reset_pin((gpio_num_t)PIN_IN3);
  gpio_reset_pin((gpio_num_t)PIN_IN2);
  gpio_reset_pin((gpio_num_t)PIN_IN1);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== rover boot ===");

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || \
    defined(ARDUINO_ESP32S3)
  roverPrepareL298nGpio();
  Serial.println("[board] S3 motor GPIO mux reset (39-44) before L298N init");
#endif

  // MPU6050 I2C — Wire.begin(15, 17)
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!imu.begin()) {
    Serial.println("IMU init failed — running motors disabled until fixed?");
  }

  // Arducam OV2640 SPI — SPI.begin(12, 13, 11, 10) = SCK, MISO, MOSI, CS
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CAM_CS);
  cam.begin();

  motors.begin();

  Serial.println("setup done, entering loop");
}

void loop() {
  uint32_t now = millis();

  if (now - lastSens >= SENSOR_MS) {
    lastSens = now;
    float gz = imu.gyroZdps();
    // lazy LPF so noise doesn't freak the motors
    yawRateFiltered = 0.85f * yawRateFiltered + 0.15f * gz;
  }

  if (now - lastCam >= CAMERA_MS) {
    lastCam = now;
    uint8_t scratch[64];
    size_t n = cam.readVisionChunk(scratch, sizeof(scratch));
    if (n > 0) {
      lastBright = cam.estimatePathBrightness(scratch, n);
    }
  }

  // target brightness ~ center of range; error drives turn
  float err = (float)lastBright - 128.0f;
  int16_t base = 140;  // forward bias

  int16_t left = computeLeft(base, err, yawRateFiltered);
  int16_t right = computeRight(base, err, yawRateFiltered);

  motors.setTankDrive(left, right);

  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.printf("[nav] bright=%u err=%.1f yaw_f=%.2f L=%d R=%d\n", lastBright, err,
                  yawRateFiltered, left, right);
  }
}
