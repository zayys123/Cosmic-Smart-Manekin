
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Arduino_LSM6DS3.h>
#include <math.h>

// =====================================================
// 1. SENSOR CONFIGURATION
// =====================================================

#define SEALEVELPRESSURE_HPA 1013.25f

#define DFR0034_Left  (35)
#define DFR0034_Right (34)

Adafruit_BME280 bme;

// =====================================================
// 2. CALIBRATION
// =====================================================

const float SOUND_CAL_GAIN = 1.0f;
const float SOUND_CAL_OFFSET_DB = 37.23f;

// =====================================================
// 3. DFR0034 CONFIGURATION
// =====================================================

const uint32_t SOUND_SAMPLE_RATE = 8000;

const uint32_t SOUND_SAMPLE_INTERVAL_US =
    1000000UL / SOUND_SAMPLE_RATE;

const float DFR_RMS_TIME_US = 125000.0f;
const float DFR_DC_TIME_US = 1000000.0f;

const float DFR_MIN_RMS_ADC_COUNTS = 0.05f;

// =====================================================
// 4. TIMING CONFIGURATION
// =====================================================

unsigned long lastIMURead = 0;
unsigned long lastBMERead = 0;
unsigned long lastCSVRead = 0;

const unsigned long IMU_INTERVAL = 10;
const unsigned long BME_INTERVAL = 500;
const unsigned long CSV_INTERVAL = 100;

uint32_t nextSampleMicros = 0;

unsigned long startMillis = 0;

// =====================================================
// 5. IMU VARIABLES
// =====================================================

float accelx = 0;
float accely = 0;
float accelz = 0;

float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

// =====================================================
// 6. BME280 VARIABLES
// =====================================================

float bme_temp = 0;
float bme_hum = 0;
float bme_pres = 0;
float bme_alt = 0;

// =====================================================
// 7. DFR0034 SENSOR STRUCTURE
// =====================================================

struct DFRSensor {

    // DC removal
    float soundDC = 0.0f;
    bool dcInitialized = false;

    // RMS
    float meanSquare = 0.0f;

    // A-weighting filter states
    float z1_1 = 0.0f;
    float z2_1 = 0.0f;

    float z1_2 = 0.0f;
    float z2_2 = 0.0f;

    float z1_3 = 0.0f;
    float z2_3 = 0.0f;

    // Sound output
    float rawDB = NAN;
    float calibratedDB = NAN;

    // Sampling
    uint32_t lastSampleMicros = 0;

    // ADC diagnostics
    uint16_t adcMin = 4095;
    uint16_t adcMax = 0;

    uint32_t clipLow = 0;
    uint32_t clipHigh = 0;

    uint32_t sampleCount = 0;
    uint32_t missedSamples = 0;
};

// Two independent sensors

DFRSensor dfrLeft;
DFRSensor dfrRight;

// =====================================================
// 8. A-WEIGHTING COEFFICIENTS
// Sampling frequency: 8 kHz
// =====================================================

// Stage 1

const float a0_1 = 0.61934873f;
const float a1_1 = 1.23869747f;
const float a2_1 = 0.61934873f;

const float b1_1 = 0.10381805f;
const float b2_1 = -0.36041744f;

// Stage 2

const float a0_2 = 1.0f;
const float a1_2 = -2.00014161f;
const float a2_2 = 1.00014162f;

const float b1_2 = -0.26438243f;
const float b2_2 = -0.60140346f;

// Stage 3

const float a0_3 = 1.0f;
const float a1_3 = -1.99985839f;
const float a2_3 = 0.99985840f;

const float b1_3 = -1.96790281f;
const float b2_3 = 0.96816037f;

// =====================================================
// 9. FUNCTION DECLARATIONS
// =====================================================

float processBiquad(
    float input,
    float a0,
    float a1,
    float a2,
    float b1,
    float b2,
    float &z1,
    float &z2
);

void processDFRSample(
    DFRSensor &sensor,
    uint16_t rawADC,
    uint32_t sampleTime
);

void readDFR0034();

void updateSoundLevel(DFRSensor &sensor);

void readBME280();

void readIMU();

void printCSV();

void resetDiagnostics(DFRSensor &sensor);

// =====================================================
// 10. BIQUAD FILTER
// =====================================================

float processBiquad(
    float input,
    float a0,
    float a1,
    float a2,
    float b1,
    float b2,
    float &z1,
    float &z2
) {

    float output = a0 * input + z1;

    z1 = a1 * input - b1 * output + z2;

    z2 = a2 * input - b2 * output;

    return output;
}

// =====================================================
// 11. DFR0034 PROCESSING
// =====================================================

void processDFRSample(
    DFRSensor &sensor,
    uint16_t rawADC,
    uint32_t sampleTime
) {

    // ADC diagnostics

    if (rawADC < sensor.adcMin) {
        sensor.adcMin = rawADC;
    }

    if (rawADC > sensor.adcMax) {
        sensor.adcMax = rawADC;
    }

    if (rawADC <= 10) {
        sensor.clipLow++;
    }

    if (rawADC >= 4085) {
        sensor.clipHigh++;
    }

    sensor.sampleCount++;

    // Actual sampling interval

    uint32_t deltaUs = SOUND_SAMPLE_INTERVAL_US;

    if (sensor.lastSampleMicros != 0) {

        deltaUs =
            sampleTime - sensor.lastSampleMicros;
    }

    sensor.lastSampleMicros = sampleTime;

    if (deltaUs == 0) {
        deltaUs = SOUND_SAMPLE_INTERVAL_US;
    }

    if (deltaUs > 4 * SOUND_SAMPLE_INTERVAL_US) {
        deltaUs = 4 * SOUND_SAMPLE_INTERVAL_US;
    }

    // DC removal

    if (!sensor.dcInitialized) {

        sensor.soundDC = rawADC;

        sensor.dcInitialized = true;

        return;
    }

    float dcAlpha =
        (float)deltaUs /
        (DFR_DC_TIME_US + deltaUs);

    sensor.soundDC +=
        dcAlpha * ((float)rawADC - sensor.soundDC);

    float centeredSignal =
        (float)rawADC - sensor.soundDC;

    // A-weighting stage 1

    float filtered = processBiquad(
        centeredSignal,
        a0_1, a1_1, a2_1,
        b1_1, b2_1,
        sensor.z1_1,
        sensor.z2_1
    );

    // A-weighting stage 2

    filtered = processBiquad(
        filtered,
        a0_2, a1_2, a2_2,
        b1_2, b2_2,
        sensor.z1_2,
        sensor.z2_2
    );

    // A-weighting stage 3

    filtered = processBiquad(
        filtered,
        a0_3, a1_3, a2_3,
        b1_3, b2_3,
        sensor.z1_3,
        sensor.z2_3
    );

    // RMS calculation

    float square = filtered * filtered;

    if (!isfinite(square)) {

        sensor.z1_1 = sensor.z2_1 = 0.0f;
        sensor.z1_2 = sensor.z2_2 = 0.0f;
        sensor.z1_3 = sensor.z2_3 = 0.0f;

        sensor.meanSquare = 0.0f;

        sensor.dcInitialized = false;

        sensor.rawDB = NAN;
        sensor.calibratedDB = NAN;

        return;
    }

    float rmsAlpha =
        (float)deltaUs /
        (DFR_RMS_TIME_US + deltaUs);

    sensor.meanSquare +=
        rmsAlpha * (square - sensor.meanSquare);

    if (sensor.meanSquare < 0.0f) {
        sensor.meanSquare = 0.0f;
    }
}

// =====================================================
// 12. READ BOTH DFR0034 SENSORS
// =====================================================

void readDFR0034() {

    uint32_t currentMicros = micros();

    int32_t lateness =
        (int32_t)(
            currentMicros - nextSampleMicros
        );

    if (lateness < 0) {
        return;
    }

    uint32_t missedSlots =
        (uint32_t)lateness /
        SOUND_SAMPLE_INTERVAL_US;

    dfrLeft.missedSamples += missedSlots;
    dfrRight.missedSamples += missedSlots;

    nextSampleMicros +=
        (missedSlots + 1) *
        SOUND_SAMPLE_INTERVAL_US;

    // DFR0034 LEFT

    uint16_t leftADC =
        analogRead(DFR0034_Left);

    uint32_t leftTime = micros();

    processDFRSample(
        dfrLeft,
        leftADC,
        leftTime
    );

    // DFR0034 RIGHT

    uint16_t rightADC =
        analogRead(DFR0034_Right);

    uint32_t rightTime = micros();

    processDFRSample(
        dfrRight,
        rightADC,
        rightTime
    );
}

// =====================================================
// 13. SOUND CALIBRATION
// =====================================================

void updateSoundLevel(DFRSensor &sensor) {

    if (millis() - startMillis < 500) {

        sensor.rawDB = NAN;
        sensor.calibratedDB = NAN;

        return;
    }

    float rms = sqrtf(sensor.meanSquare);

    if (
        isfinite(rms) &&
        rms >= DFR_MIN_RMS_ADC_COUNTS
    ) {

        float rawLevel =
            20.0f * log10f(rms);

        sensor.rawDB = rawLevel;

        // Calibration formula

        sensor.calibratedDB =
            SOUND_CAL_GAIN * rawLevel +
            SOUND_CAL_OFFSET_DB;

    } else {

        sensor.rawDB = NAN;
        sensor.calibratedDB = NAN;
    }
}

// =====================================================
// 14. BME280
// =====================================================

void readBME280() {

    bme_temp = bme.readTemperature();

    bme_hum = bme.readHumidity();

    bme_pres =
        bme.readPressure() / 100.0f;

    bme_alt =
        bme.readAltitude(
            SEALEVELPRESSURE_HPA
        );
}

// =====================================================
// 15. IMU
// =====================================================

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

// =====================================================
// 16. RESET DIAGNOSTICS
// =====================================================

void resetDiagnostics(DFRSensor &sensor) {

    sensor.adcMin = 4095;
    sensor.adcMax = 0;

    sensor.clipLow = 0;
    sensor.clipHigh = 0;

    sensor.sampleCount = 0;
    sensor.missedSamples = 0;
}

// =====================================================
// 17. CSV OUTPUT
// =====================================================

void printCSV() {

    updateSoundLevel(dfrLeft);
    updateSoundLevel(dfrRight);

    // IMU

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

    // BME280

    Serial.print(bme_temp, 2);
    Serial.print(",");

    Serial.print(bme_hum, 2);
    Serial.print(",");

    Serial.print(bme_pres, 2);
    Serial.print(",");

    Serial.print(bme_alt, 2);
    Serial.print(",");

    // DFR0034 LEFT

    Serial.print(dfrLeft.rawDB, 3);
    Serial.print(",");

    Serial.print(dfrLeft.calibratedDB, 3);
    Serial.print(",");

    // DFR0034 RIGHT

    Serial.print(dfrRight.rawDB, 3);
    Serial.print(",");

    Serial.print(dfrRight.calibratedDB, 3);
    Serial.print(",");

    // LEFT DIAGNOSTICS

    Serial.print(dfrLeft.adcMin);
    Serial.print(",");

    Serial.print(dfrLeft.adcMax);
    Serial.print(",");

    Serial.print(dfrLeft.clipLow);
    Serial.print(",");

    Serial.print(dfrLeft.clipHigh);
    Serial.print(",");

    Serial.print(dfrLeft.sampleCount);
    Serial.print(",");

    Serial.print(dfrLeft.missedSamples);
    Serial.print(",");

    // RIGHT DIAGNOSTICS

    Serial.print(dfrRight.adcMin);
    Serial.print(",");

    Serial.print(dfrRight.adcMax);
    Serial.print(",");

    Serial.print(dfrRight.clipLow);
    Serial.print(",");

    Serial.print(dfrRight.clipHigh);
    Serial.print(",");

    Serial.print(dfrRight.sampleCount);
    Serial.print(",");

    Serial.print(dfrRight.missedSamples);
    Serial.print(",");

    Serial.println(millis());

    resetDiagnostics(dfrLeft);
    resetDiagnostics(dfrRight);
}

// =====================================================
// 18. SETUP
// =====================================================

void setup() {

    Serial.begin(115200);

    // ADC configuration

    analogReadResolution(12);

    pinMode(DFR0034_Left, INPUT);
    pinMode(DFR0034_Right, INPUT);

    analogSetPinAttenuation(
        DFR0034_Left,
        ADC_11db
    );

    analogSetPinAttenuation(
        DFR0034_Right,
        ADC_11db
    );

    // IMU initialization

    if (!IMU.begin()) {

        Serial.println("ERR:IMU_FAIL");

        while (1) {
            delay(1000);
        }
    }

    // BME280 initialization

    if (!bme.begin(0x76)) {

        Serial.println("ERR:BME_FAIL");

        while (1) {
            delay(1000);
        }
    }

    // Non-blocking BME280 operation

    bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::SAMPLING_X1,
        Adafruit_BME280::FILTER_OFF,
        Adafruit_BME280::STANDBY_MS_500
    );

    // Initialize timing

    startMillis = millis();

    nextSampleMicros = micros();

    // CSV HEADER

    Serial.println(
        "accelx,accely,accelz,"
        "gyrox,gyroy,gyroz,"
        "bme_temp,bme_hum,bme_pres,bme_alt,"
        "dfr_left_raw_db,dfr_left_dba,"
        "dfr_right_raw_db,dfr_right_dba,"
        "left_adc_min,left_adc_max,"
        "left_clip_low,left_clip_high,"
        "left_samples,left_missed,"
        "right_adc_min,right_adc_max,"
        "right_clip_low,right_clip_high,"
        "right_samples,right_missed,"
        "timestamp_ms"
    );
}

// =====================================================
// 19. MAIN LOOP
// =====================================================

void loop() {

    // DFR0034 sampling

    readDFR0034();

    unsigned long currentMillis = millis();

    // BME280: 2 Hz

    if (
        currentMillis - lastBMERead >= BME_INTERVAL
    ) {

        lastBMERead = currentMillis;

        readBME280();
    }

    // IMU: 100 Hz

    if (
        currentMillis - lastIMURead >= IMU_INTERVAL
    ) {

        lastIMURead = currentMillis;

        readIMU();
    }

    // CSV: 10 Hz

    if (
        currentMillis - lastCSVRead >= CSV_INTERVAL
    ) {

        lastCSVRead = currentMillis;

        printCSV();
    }
}
