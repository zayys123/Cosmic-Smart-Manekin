#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino_LSM6DS3.h>

// ============================================================
// 1. KONFIGURASI PIN & KONSTANTA
// ============================================================
#define SEALEVELPRESSURE_HPA 1013.25f

// Pin penghubung UART Raspberry Pi
#define PI_RX 3
#define PI_TX 1
#define PI_BAUDRATE 115200

// Pin sensor suara SEN0232 (sesuai kode kedua)
#define SEN0232_Left  35
#define SEN0232_Right 34

const float SEN0232_SENSITIVITY = 50.0f;

Adafruit_BME280 bme;

// ============================================================
// 2. VARIABEL DATA SENSOR
// ============================================================
float accelx = 0.0f, accely = 0.0f, accelz = 0.0f;
float gyrox = 0.0f, gyroy = 0.0f, gyroz = 0.0f;

float bme_temp = 0.0f;
float bme_hum = 0.0f;
float bme_pres = 0.0f;
float bme_alt = 0.0f;

// Left sensor
float sen0232_left_voltage = 0.0f;
float sen0232_left_db = 0.0f;

// Right sensor
float sen0232_right_voltage = 0.0f;
float sen0232_right_db = 0.0f;

// ============================================================
// 3. INTERVAL AKUISISI (NON-BLOCKING)
// ============================================================
unsigned long lastIMURead = 0;
unsigned long lastBMERead = 0;
unsigned long lastSoundRead = 0;

const unsigned long IMU_INTERVAL = 10;    // 100 Hz (10 ms)
const unsigned long BME_INTERVAL = 500;   // 2 Hz (500 ms)
const unsigned long SOUND_INTERVAL = 10;  // 100 Hz (10 ms)

// ============================================================
// 4. DEKLARASI FUNGSI
// ============================================================
void readSEN0232();
void readBME280();
void readIMU();
void printCSV();

// ============================================================
// 5. FUNGSI PEMBACAAN SENSOR
// ============================================================
void readSEN0232() {
    // SEN0232 LEFT (GPIO 35)
    uint32_t left_mV = analogReadMilliVolts(SEN0232_Left);
    sen0232_left_voltage = left_mV / 1000.0f;
    sen0232_left_db = sen0232_left_voltage * SEN0232_SENSITIVITY;
    sen0232_left_db = constrain(sen0232_left_db, 0.0f, 130.0f);

    // SEN0232 RIGHT (GPIO 34)
    uint32_t right_mV = analogReadMilliVolts(SEN0232_Right);
    sen0232_right_voltage = right_mV / 1000.0f;
    sen0232_right_db = sen0232_right_voltage * SEN0232_SENSITIVITY;
    sen0232_right_db = constrain(sen0232_right_db, 0.0f, 130.0f);
}

void readBME280() {
    bme.takeForcedMeasurement();
    bme_temp = bme.readTemperature();
    bme_hum = bme.readHumidity();
    bme_pres = bme.readPressure() / 100.0f;
    bme_alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
}

void readIMU() {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        IMU.readAcceleration(accelx, accely, accelz);
        IMU.readGyroscope(gyrox, gyroy, gyroz);
    }
}

// ============================================================
// 6. PENGIRIMAN DATA CSV KE RASPBERRY PI (12 KOLOM)
// ============================================================
void printCSV() {
    // IMU (6 kolom)
    Serial1.print(accelx, 3); Serial1.print(",");
    Serial1.print(accely, 3); Serial1.print(",");
    Serial1.print(accelz, 3); Serial1.print(",");
    Serial1.print(gyrox, 3);  Serial1.print(",");
    Serial1.print(gyroy, 3);  Serial1.print(",");
    Serial1.print(gyroz, 3);  Serial1.print(",");

    // BME280 (4 kolom)
    Serial1.print(bme_temp, 2); Serial1.print(",");
    Serial1.print(bme_hum, 2);  Serial1.print(",");
    Serial1.print(bme_pres, 2); Serial1.print(",");
    Serial1.print(bme_alt, 2);  Serial1.print(",");

    // SEN0232 dBA (2 kolom)
    Serial1.print(sen0232_left_db, 2);
    Serial1.print(",");
    Serial1.println(sen0232_right_db, 2);
}

// ============================================================
// 7. SETUP
// ============================================================
void setup() {
    // Debug melalui USB
    Serial.begin(115200);
    
    // Komunikasi UART ke Raspberry Pi
    Serial1.begin(PI_BAUDRATE, SERIAL_8N1, PI_RX, PI_TX);

    // Konfigurasi ADC 12-bit
    analogReadResolution(12);
    pinMode(SEN0232_Left, INPUT);
    pinMode(SEN0232_Right, INPUT);
    analogSetPinAttenuation(SEN0232_Left, ADC_11db);
    analogSetPinAttenuation(SEN0232_Right, ADC_11db);

    // Inisialisasi IMU
    if (!IMU.begin()) {
        Serial.println("ERR:IMU_FAIL");
        Serial1.println("ERR:IMU_FAIL");
        while (1) {
            delay(1000);
        }
    }

    // Inisialisasi BME280
    if (!bme.begin(0x76)) {
        Serial.println("ERR:BME_FAIL");
        Serial1.println("ERR:BME_FAIL");
        while (1) {
            delay(1000);
        }
    }

    bme.setSampling(
        Adafruit_BME280::MODE_FORCED,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_OFF
    );
}

// ============================================================
// 8. MAIN LOOP
// ============================================================
void loop() {
    unsigned long currentMillis = millis();

    // 1. Baca BME280 (2 Hz)
    if (currentMillis - lastBMERead >= BME_INTERVAL) {
        lastBMERead = currentMillis;
        readBME280();
    }

    // 2. Baca SEN0232 (100 Hz)
    if (currentMillis - lastSoundRead >= SOUND_INTERVAL) {
        lastSoundRead = currentMillis;
        readSEN0232();
    }

    // 3. Baca IMU & Kirim CSV ke Raspberry Pi (100 Hz)
    if (currentMillis - lastIMURead >= IMU_INTERVAL) {
        lastIMURead = currentMillis;
        readIMU();
        printCSV();
    }
}