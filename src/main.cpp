/**
 * ESP32-S3 rover — camera obstacle avoidance using ArduCAM OV2640 (SPI FIFO + I2C sensor).
 * Wire @ SDA/SCL talks to MPU6050 (0x68) and OV2640 regs through the ArduCAM bridge;
 * SPI pushes CS low and bursts pixel data from the camera FIFO after each capture.
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

// --- Pins (KiCad / ESP32-S3) ---
static const uint8_t PIN_IN1 = 44;
static const uint8_t PIN_IN2 = 43;
static const uint8_t PIN_IN3 = 42;
static const uint8_t PIN_IN4 = 41;
static const uint8_t PIN_ENA = 40;
static const uint8_t PIN_ENB = 39;
static const uint8_t PIN_SDA = 15;
static const uint8_t PIN_SCL = 17;
static const uint8_t PIN_SCK = 12;
static const uint8_t PIN_MISO = 13;
static const uint8_t PIN_MOSI = 11;
static const uint8_t PIN_CAM_CS = 10;

// --- Obstacle detection (RGB565 "red blob" in middle horizontal band) ---
// Each pixel is 16 bits: 5 bits R, 6 bits G, 5 bits B (no separate bytes — packed in one uint16).
#define DETECTION_THRESHOLD_PERCENT 20  // call it an obstacle if >= this % of band pixels look "red"
#define RED_MIN_5BIT 22                 // R must be at least this (0–31 scale)
#define GREEN_MAX_6BIT 14               // G must be at most this (0–63 scale)
#define BLUE_MAX_5BIT 14                // B must be at most this (0–31 scale)

// --- Driving / timing ---
static const int16_t CRUISE_BASE_SPEED = 130;
static const int16_t TURN_LEFT_WHEEL = 150;
static const int16_t TURN_RIGHT_WHEEL = -150;
static const uint32_t SENSOR_MS = 20;
static const uint32_t OBSTACLE_CHECK_MS = 40;
static const uint32_t SERIAL_STATUS_MS = 400;
// Don't print faster than this — still feels "live" but won't overwhelm Serial Monitor.
static const uint32_t CAMERA_DEBUG_MIN_MS = 150;
// How long we sit in STOP (motors off) before we start AVOID turn — easy to show on demo day.
static const uint32_t STOP_HOLD_MS = 200;

SensorI2C imu(PIN_SDA, PIN_SCL);
MotorDriverPWM motors(PIN_ENA, PIN_ENB, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4);
CameraSPI cam(PIN_CAM_CS);

uint32_t lastSens = 0;
uint32_t lastObs = 0;
uint32_t lastSerial = 0;
uint32_t lastCameraPrint = 0;

float yawRateFiltered = 0.0f;

// Navigation state machine (FORWARD -> STOP -> AVOID -> FORWARD)
enum RoverState : uint8_t { STATE_FORWARD = 0, STATE_STOP = 1, STATE_AVOID = 2 };
static RoverState roverState = STATE_FORWARD;
static uint32_t stopStateEnteredMs = 0;

bool isObstacleDetected = false;
static float lastRedBandPercent = 0.0f;  // for Serial debug

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

/**
 * Grab one frame from the ArduCAM FIFO (SPI), then count "red-looking" pixels in the
 * middle horizontal band (middle third of rows). Returns false if capture failed.
 *
 * RGB565 math (one 16-bit value per pixel):
 *   bits 15..11 -> Red   (5 bits): shift right by 11, then mask 0x1F
 *   bits 10..5  -> Green (6 bits): shift right by 5,  mask 0x3F
 *   bits 4..0   -> Blue  (5 bits): mask 0x1F (no shift)
 * We compare those to RED_MIN / GREEN_MAX / BLUE_MAX to catch "mostly red" obstacles.
 */
bool detectObstacle(CameraSPI &camera, bool &outObstacle, float &outRedPercent) {
  outObstacle = false;
  outRedPercent = 0.0f;

  if (!camera.captureFrameQCIF_DMA()) {
    Serial.println("[detectObstacle] FIFO capture failed");
    return false;
  }

  const uint16_t *pixels = (const uint16_t *)camera.frameBuffer();
  uint16_t imgW = camera.width();
  uint16_t imgH = camera.height();
  if (!pixels || imgW == 0 || imgH < 3) {
    return false;
  }

  // Middle horizontal band: rows [H/3 .. 2H/3) — where we expect obstacles in front of the rover.
  uint16_t rowStart = imgH / 3;
  uint16_t rowEnd = (2u * imgH) / 3;

  uint32_t redCount = 0;
  uint32_t totalInBand = 0;

  for (uint16_t y = rowStart; y < rowEnd; y++) {
    const uint16_t *row = pixels + (size_t)y * imgW;
    for (uint16_t x = 0; x < imgW; x++) {
      uint16_t p = row[x];
      // Split RGB565 into R, G, B using the bit layout above.
      uint8_t r = (uint8_t)((p >> 11) & 0x1F);
      uint8_t g = (uint8_t)((p >> 5) & 0x3F);
      uint8_t b = (uint8_t)(p & 0x1F);

      totalInBand++;
      if (r >= RED_MIN_5BIT && g <= GREEN_MAX_6BIT && b <= BLUE_MAX_5BIT) {
        redCount++;
      }
    }
  }

  float pct = 0.0f;
  if (totalInBand > 0) {
    pct = 100.0f * (float)redCount / (float)totalInBand;
  }
  outRedPercent = pct;
  outObstacle = (pct >= (float)DETECTION_THRESHOLD_PERCENT);
  return true;
}

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || \
    defined(ARDUINO_ESP32S3)
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
  Serial.println("\n=== rover boot (ArduCAM obstacle + state machine) ===");

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV) || \
    defined(ARDUINO_ESP32S3)
  roverPrepareL298nGpio();
#endif

  // Shared I2C bus: MPU6050 @ 0x68, OV2640 sensor regs via ArduCAM @ 0x30 (7-bit addresses).
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!imu.begin()) {
    Serial.println("IMU init failed");
  }

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CAM_CS);
  if (!cam.begin()) {
    Serial.println("[fatal] camera init failed");
  }

  motors.begin();

  Serial.println("States: FORWARD -> STOP (red >= threshold) -> AVOID (turn) -> FORWARD");
}

void loop() {
  uint32_t now = millis();

  if (now - lastSens >= SENSOR_MS) {
    lastSens = now;
    float gz = imu.gyroZdps();
    yawRateFiltered = 0.85f * yawRateFiltered + 0.15f * gz;
  }

  if (now - lastObs >= OBSTACLE_CHECK_MS) {
    lastObs = now;
    bool obs = false;
    float redPct = 0.0f;
    if (detectObstacle(cam, obs, redPct)) {
      isObstacleDetected = obs;
      lastRedBandPercent = redPct;

      if (now - lastCameraPrint >= CAMERA_DEBUG_MIN_MS) {
        lastCameraPrint = now;
        Serial.print("Camera: red pixels ~ ");
        Serial.print(redPct, 1);
        Serial.print("% (threshold ");
        Serial.print(DETECTION_THRESHOLD_PERCENT);
        Serial.println("%)  ");
      }

      if (obs && roverState == STATE_FORWARD) {
        roverState = STATE_STOP;
        stopStateEnteredMs = millis();
        Serial.println("State -> STOP (cut motors briefly)");
      }
    }
  }

  float err = 0.0f;
  int16_t leftCruise = computeLeft(CRUISE_BASE_SPEED, err, yawRateFiltered);
  int16_t rightCruise = computeRight(CRUISE_BASE_SPEED, err, yawRateFiltered);

  switch (roverState) {
    case STATE_FORWARD:
      motors.setTankDrive(leftCruise, rightCruise);
      break;

    case STATE_STOP:
      motors.stop();
      if (!isObstacleDetected) {
        roverState = STATE_FORWARD;
        Serial.println("State -> FORWARD (obstacle cleared while stopped)");
      } else if ((now - stopStateEnteredMs) >= STOP_HOLD_MS) {
        roverState = STATE_AVOID;
        Serial.println("State -> AVOID (turn right to get blob out of view)");
      }
      break;

    case STATE_AVOID:
      motors.setTankDrive(TURN_LEFT_WHEEL, TURN_RIGHT_WHEEL);
      if (!isObstacleDetected) {
        roverState = STATE_FORWARD;
        Serial.println("State -> FORWARD (blob gone)");
      }
      break;

    default:
      roverState = STATE_FORWARD;
      break;
  }

  if (now - lastSerial >= SERIAL_STATUS_MS) {
    lastSerial = now;
    Serial.print("[nav] state=");
    switch (roverState) {
      case STATE_FORWARD:
        Serial.print("FORWARD");
        break;
      case STATE_STOP:
        Serial.print("STOP");
        break;
      case STATE_AVOID:
        Serial.print("AVOID");
        break;
      default:
        Serial.print("?");
        break;
    }
    Serial.print("  lastRed=");
    Serial.print(lastRedBandPercent, 1);
    Serial.println("%");
  }
}
