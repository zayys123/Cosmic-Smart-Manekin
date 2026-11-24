#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino_LSM6DS3.h>

#define SEALEVELPRESSURE_HPA 1013.25f
#define SEN0232_Left  (35)
#define SEN0232_Right (34)

const float SEN0232_SENSITIVITY = 50.0f;

Adafruit_BME280 bme;

float accelx = 0;
float accely = 0;
float accelz = 0;

float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

float bme_temp = 0;
float bme_hum = 0;
float bme_pres = 0;
float bme_alt = 0;

// Left sensor
float sen0232_left_voltage = 0;
float sen0232_left_db = 0;

// Right sensor
float sen0232_right_voltage = 0;
float sen0232_right_db = 0;

unsigned long lastIMURead = 0;
unsigned long lastBMERead = 0;
unsigned long lastSoundRead = 0;

const unsigned long IMU_INTERVAL = 10;
const unsigned long BME_INTERVAL = 500;
const unsigned long SOUND_INTERVAL = 10;

void readSEN0232();
void readBME280();
void readIMU();
void printCSV();

void readSEN0232() {
    // -----------------------------------------------
    // SEN0232 LEFT
    // -----------------------------------------------
    uint32_t left_mV =
        analogReadMilliVolts(SEN0232_Left);
    sen0232_left_voltage =
        left_mV / 1000.0f;
    sen0232_left_db =
        sen0232_left_voltage *
        SEN0232_SENSITIVITY;
    sen0232_left_db = constrain(
        sen0232_left_db,
        30.0f,
        130.0f
    );

    // -----------------------------------------------
    // SEN0232 RIGHT
    // -----------------------------------------------
    uint32_t right_mV =
        analogReadMilliVolts(SEN0232_Right);
    sen0232_right_voltage =
        right_mV / 1000.0f;
    sen0232_right_db =
        sen0232_right_voltage *
        SEN0232_SENSITIVITY;
    sen0232_right_db = constrain(
        sen0232_right_db,
        30.0f,
        130.0f
    );
}

void readBME280() {
    bme.takeForcedMeasurement();
    bme_temp = bme.readTemperature();
    bme_hum = bme.readHumidity();
    bme_pres =
        bme.readPressure() / 100.0f;
    bme_alt =
        bme.readAltitude(
            SEALEVELPRESSURE_HPA
        );
}

void readIMU() {
    if (
        IMU.accelerationAvailable() &&
        IMU.gyroscopeAvailable()
    ) {
        IMU.readAcceleration(
            accelx,
            accely,
            accelz
        );
        IMU.readGyroscope(
            gyrox,
            gyroy,
            gyroz
        );
    }
}

void setup() {
    Serial.begin(115200);
    // ADC CONFIGURATION
    analogReadResolution(12);
    pinMode(SEN0232_Left, INPUT);
    pinMode(SEN0232_Right, INPUT);
    analogSetPinAttenuation(
        SEN0232_Left,
        ADC_11db
    );
    analogSetPinAttenuation(
        SEN0232_Right,
        ADC_11db
    );
    // IMU INITIALIZATION
    if (!IMU.begin()) {
        Serial.println("ERR:IMU_FAIL");
        while (1) {
            delay(1000);
        }
    }

    // BME280 INITIALIZATION
    if (!bme.begin(0x76)) {
        Serial.println("ERR:BME_FAIL");
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
    Serial.println(
        "accelx,accely,accelz,"
        "gyrox,gyroy,gyroz,"
        "bme_temp,bme_hum,bme_pres,bme_alt,"
        "sen0232_left_voltage,"
        "sen0232_left_db,"
        "sen0232_right_voltage,"
        "sen0232_right_db"
    );
}

void loop() {
    unsigned long currentMillis = millis();
    if (
        currentMillis - lastBMERead >= BME_INTERVAL
    ) {
        lastBMERead = currentMillis;
        readBME280();
    }
    if (
        currentMillis - lastSoundRead >= SOUND_INTERVAL
    ) {
        lastSoundRead = currentMillis;
        readSEN0232();
    }
    if (
        currentMillis - lastIMURead >= IMU_INTERVAL
    ) {
        lastIMURead = currentMillis;
        readIMU();
        printCSV();
    }
}

void printCSV() {

    Serial.print(accelx, 3);
    Serial.print(",");

    Serial.print(accely, 3);
    Serial.print(",");

    Serial.print(accelz, 3);
    Serial.print(",");

    Serial.print(gyrox, 3);
    Serial.print(",");

    Serial.print(gyroy, 3);
    Serial.print(",");

    Serial.print(gyroz, 3);
    Serial.print(",");

    Serial.print(bme_temp, 2);
    Serial.print(",");

    Serial.print(bme_hum, 2);
    Serial.print(",");

    Serial.print(bme_pres, 2);
    Serial.print(",");

    Serial.print(bme_alt, 2);
    Serial.print(",");

    Serial.print(sen0232_left_voltage, 4);
    Serial.print(",");

    Serial.print(sen0232_left_db, 2);
    Serial.print(",");

    Serial.print(sen0232_right_voltage, 4);
    Serial.print(",");

    Serial.println(sen0232_right_db, 2);
}
