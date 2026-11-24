#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>

// ============================================================
// 1. KONFIGURASI UART RASPBERRY PI
// ============================================================
// Catatan: GPIO 1 & 3 berbagi jalur dengan USB Serial (UART0).
// Jika terjadi bentrok data ke Raspi, pertimbangkan pindah ke RX=16, TX=17.
#define UART_RX_PIN 3
#define UART_TX_PIN 1
#define UART_BAUDRATE 115200

// ============================================================
// 2. KONFIGURASI SENSOR & KALIBRASI
// ============================================================
#define SEALEVELPRESSURE_HPA 1013.25f

// Sesuai mapping pin kode pertama (Gateway)
#define SOUND_LEFT_PIN  35
#define SOUND_RIGHT_PIN 34

// Parameter Kalibrasi Suara (dBA)
const float SOUND_CAL_GAIN = 1.0f;
const float SOUND_CAL_OFFSET_DB = 37.23f;

// Konfigurasi Sampling Audio 8 kHz
const uint32_t SOUND_SAMPLE_RATE = 8000;
const uint32_t SOUND_SAMPLE_INTERVAL_US = 1000000UL / SOUND_SAMPLE_RATE;
const float DFR_RMS_TIME_US = 125000.0f;
const float DFR_DC_TIME_US = 1000000.0f;
const float DFR_MIN_RMS_ADC_COUNTS = 0.05f;

Adafruit_BME280 bme;

// ============================================================
// 3. INTERVAL AKUISISI
// ============================================================
const unsigned long IMU_INTERVAL_MS = 10;   // 100 Hz
const unsigned long BME_INTERVAL_MS = 500;  // 2 Hz

unsigned long lastIMURead = 0;
unsigned long lastBMERead = 0;
unsigned long startMillis = 0;
uint32_t nextSampleMicros = 0;

// ============================================================
// 4. VARIABEL & STRUKTUR SENSOR
// ============================================================
// IMU
float accel_x = 0.0f;
float accel_y = 0.0f;
float accel_z = 0.0f;

float gyro_x = 0.0f;
float gyro_y = 0.0f;
float gyro_z = 0.0f;

// BME280
float bme_temperature = 0.0f;
float bme_humidity = 0.0f;
float bme_pressure = 0.0f;
float bme_altitude = 0.0f;

// Status sensor
bool imu_ready = false;
bool bme_ready = false;

// Struktur Filter & RMS Sound Sensor
struct SoundSensor {
  float soundDC = 0.0f;
  bool dcInitialized = false;
  float meanSquare = 0.0f;

  // State Filter A-weighting
  float z1_1 = 0.0f, z2_1 = 0.0f;
  float z1_2 = 0.0f, z2_2 = 0.0f;
  float z1_3 = 0.0f, z2_3 = 0.0f;

  float rawDB = 0.0f;
  float calibratedDB = 0.0f;
  uint32_t lastSampleMicros = 0;
};

SoundSensor soundLeft;
SoundSensor soundRight;

// ============================================================
// 5. KOEFISIEN FILTER A-WEIGHTING (Fs = 8 kHz)
// ============================================================
// Stage 1
const float a0_1 = 0.61934873f, a1_1 = 1.23869747f, a2_1 = 0.61934873f;
const float b1_1 = 0.10381805f, b2_1 = -0.36041744f;

// Stage 2
const float a0_2 = 1.0f, a1_2 = -2.00014161f, a2_2 = 1.00014162f;
const float b1_2 = -0.26438243f, b2_2 = -0.60140346f;

// Stage 3
const float a0_3 = 1.0f, a1_3 = -1.99985839f, a2_3 = 0.99985840f;
const float b1_3 = -1.96790281f, b2_3 = 0.96816037f;

// ============================================================
// 6. PEMROSESAN SINYAL AUDIO (BIQUAD & RMS)
// ============================================================
float processBiquad(float input, float a0, float a1, float a2, float b1, float b2, float &z1, float &z2) {
  float output = a0 * input + z1;
  z1 = a1 * input - b1 * output + z2;
  z2 = a2 * input - b2 * output;
  return output;
}

void processSoundSample(SoundSensor &sensor, uint16_t rawADC, uint32_t sampleTime) {
  uint32_t deltaUs = SOUND_SAMPLE_INTERVAL_US;
  if (sensor.lastSampleMicros != 0) {
    deltaUs = sampleTime - sensor.lastSampleMicros;
  }
  sensor.lastSampleMicros = sampleTime;

  if (deltaUs == 0) deltaUs = SOUND_SAMPLE_INTERVAL_US;
  if (deltaUs > 4 * SOUND_SAMPLE_INTERVAL_US) deltaUs = 4 * SOUND_SAMPLE_INTERVAL_US;

  // Penghilangan DC Offset
  if (!sensor.dcInitialized) {
    sensor.soundDC = rawADC;
    sensor.dcInitialized = true;
    return;
  }

  float dcAlpha = (float)deltaUs / (DFR_DC_TIME_US + deltaUs);
  sensor.soundDC += dcAlpha * ((float)rawADC - sensor.soundDC);
  float centeredSignal = (float)rawADC - sensor.soundDC;

  // Filtering A-weighting 3 Tingkat
  float filtered = processBiquad(centeredSignal, a0_1, a1_1, a2_1, b1_1, b2_1, sensor.z1_1, sensor.z2_1);
  filtered = processBiquad(filtered, a0_2, a1_2, a2_2, b1_2, b2_2, sensor.z1_2, sensor.z2_2);
  filtered = processBiquad(filtered, a0_3, a1_3, a2_3, b1_3, b2_3, sensor.z1_3, sensor.z2_3);

  // Kalkulasi Mean Square
  float square = filtered * filtered;
  if (!isfinite(square)) {
    sensor.z1_1 = sensor.z2_1 = 0.0f;
    sensor.z1_2 = sensor.z2_2 = 0.0f;
    sensor.z1_3 = sensor.z2_3 = 0.0f;
    sensor.meanSquare = 0.0f;
    sensor.dcInitialized = false;
    return;
  }

  float rmsAlpha = (float)deltaUs / (DFR_RMS_TIME_US + deltaUs);
  sensor.meanSquare += rmsAlpha * (square - sensor.meanSquare);
  if (sensor.meanSquare < 0.0f) sensor.meanSquare = 0.0f;
}

void sampleSoundSensors() {
  uint32_t currentMicros = micros();
  int32_t lateness = (int32_t)(currentMicros - nextSampleMicros);

  if (lateness < 0) return;

  uint32_t missedSlots = (uint32_t)lateness / SOUND_SAMPLE_INTERVAL_US;
  nextSampleMicros += (missedSlots + 1) * SOUND_SAMPLE_INTERVAL_US;

  // Baca Channel Kiri (GPIO 34)
  uint16_t leftADC = analogRead(SOUND_LEFT_PIN);
  processSoundSample(soundLeft, leftADC, micros());

  // Baca Channel Kanan (GPIO 35)
  uint16_t rightADC = analogRead(SOUND_RIGHT_PIN);
  processSoundSample(soundRight, rightADC, micros());
}

void updateSoundLevel(SoundSensor &sensor) {
  // Tunggu 500ms pertama agar DC filter stabil
  if (millis() - startMillis < 500) {
    sensor.rawDB = 0.0f;
    sensor.calibratedDB = 0.0f;
    return;
  }

  float rms = sqrtf(sensor.meanSquare);
  if (isfinite(rms) && rms >= DFR_MIN_RMS_ADC_COUNTS) {
    float rawLevel = 20.0f * log10f(rms);
    sensor.rawDB = rawLevel;
    sensor.calibratedDB = (SOUND_CAL_GAIN * rawLevel) + SOUND_CAL_OFFSET_DB;
  } else {
    // Fallback nilai minimum terkalibrasi saat kondisi sangat hening
    sensor.rawDB = 20.0f * log10f(DFR_MIN_RMS_ADC_COUNTS);
    sensor.calibratedDB = (SOUND_CAL_GAIN * sensor.rawDB) + SOUND_CAL_OFFSET_DB;
  }
}

// ============================================================
// 7. PEMBACAAN IMU & BME280
// ============================================================
void readIMU() {
  if (!imu_ready) return;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accel_x, accel_y, accel_z);
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x, gyro_y, gyro_z);
  }
}

void readBME280() {
  if (!bme_ready) return;

  bme_temperature = bme.readTemperature();
  bme_humidity = bme.readHumidity();
  bme_pressure = bme.readPressure() / 100.0f;
  bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

// ============================================================
// 8. PENGIRIMAN DATA CSV KE RASPBERRY PI
// ============================================================
void sendTelemetry() {
  updateSoundLevel(soundLeft);
  updateSoundLevel(soundRight);

  // Format wajib 12 kolom ke Raspberry Pi:
  // accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z,
  // temperature, humidity, pressure, altitude,
  // sound_left_dba, sound_right_dba

  Serial1.print(accel_x, 3); Serial1.print(",");
  Serial1.print(accel_y, 3); Serial1.print(",");
  Serial1.print(accel_z, 3); Serial1.print(",");
  Serial1.print(gyro_x, 3);  Serial1.print(",");
  Serial1.print(gyro_y, 3);  Serial1.print(",");
  Serial1.print(gyro_z, 3);  Serial1.print(",");
  Serial1.print(bme_temperature, 2); Serial1.print(",");
  Serial1.print(bme_humidity, 2);    Serial1.print(",");
  Serial1.print(bme_pressure, 2);    Serial1.print(",");
  Serial1.print(bme_altitude, 2);    Serial1.print(",");
  Serial1.print(soundLeft.calibratedDB, 3); Serial1.print(",");
  Serial1.println(soundRight.calibratedDB, 3);
}

// ============================================================
// 9. SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  delay(1000);

  Serial.println("\n================================");
  Serial.println(" COSMIC AURORA v3.2.0 (CALIBRATED)");
  Serial.println(" ESP32 SENSOR GATEWAY");
  Serial.println("================================");

  // Konfigurasi ADC 12-bit
  analogReadResolution(12);
  analogSetPinAttenuation(SOUND_LEFT_PIN, ADC_11db);
  analogSetPinAttenuation(SOUND_RIGHT_PIN, ADC_11db);
  pinMode(SOUND_LEFT_PIN, INPUT);
  pinMode(SOUND_RIGHT_PIN, INPUT);

  Wire.begin();

  // Inisialisasi IMU
  Serial.print("[IMU] Initializing... ");
  if (IMU.begin()) {
    imu_ready = true;
    Serial.println("OK");
  } else {
    imu_ready = false;
    Serial.println("FAILED");
  }

  // Inisialisasi BME280 (0x76 atau 0x77)
  Serial.print("[BME280] Initializing... ");
  if (bme.begin(0x76, &Wire)) {
    bme_ready = true;
    Serial.println("OK at 0x76");
  } else if (bme.begin(0x77, &Wire)) {
    bme_ready = true;
    Serial.println("OK at 0x77");
  } else {
    bme_ready = false;
    Serial.println("FAILED");
  }

  // Set BME280 ke MODE_NORMAL agar tidak memblokir sampling audio
  if (bme_ready) {
    bme.setSampling(
      Adafruit_BME280::MODE_NORMAL,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::FILTER_OFF,
      Adafruit_BME280::STANDBY_MS_500
    );
  }

  Serial.println("--------------------------------");
  Serial.print("UART1 TX: GPIO"); Serial.println(UART_TX_PIN);
  Serial.print("UART1 RX: GPIO"); Serial.println(UART_RX_PIN);
  Serial.println("Starting calibrated telemetry...");

  startMillis = millis();
  nextSampleMicros = micros();
}

// ============================================================
// 10. MAIN LOOP
// ============================================================
void loop() {
  // 1. Sampling Audio Berkelanjutan (8 kHz)
  sampleSoundSensors();

  unsigned long currentMillis = millis();

  // 2. Pembacaan IMU & Pengiriman Telemetry (100 Hz)
  if (currentMillis - lastIMURead >= IMU_INTERVAL_MS) {
    lastIMURead = currentMillis;
    readIMU();
    sendTelemetry();
  }

  // 3. Pembacaan BME280 (2 Hz)
  if (currentMillis - lastBMERead >= BME_INTERVAL_MS) {
    lastBMERead = currentMillis;
    readBME280();
  }
}
