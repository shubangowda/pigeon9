// sensors.cpp
// Handles all communication with the hardware: the IMU, the ADXL375 high-g accelerometer, the barometer, and the GPS.
// It returns clean readings (acceleration, pressure altitude, position) to the rest of the code.
// No other file talks to the sensor chips directly.

// Built-in Arduino stuff: Serial printing, delay(), and the I2C bus (Wire)
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Libraries for each sensor chip (installed through PlatformIO or the Arduino library manager)
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_ADXL375.h>
#include <Adafruit_BMP3XX.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Our own menu from sensors.h
#include "sensors.h"

// One object per sensor chip ("static" keeps them private to this file)
static Adafruit_LSM6DSOX imu;
static Adafruit_ADXL375 highG = Adafruit_ADXL375(12345);
static Adafruit_BMP3XX baro;
static SFE_UBLOX_GNSS gps;

// Pressure at the pad in Pa, measured by sensorsZeroBaro (101325 is just the standard sea level guess until then)
static float groundPressure = 101325.0f;

bool sensorsBegin() {

  // Start the I2C bus at 400 kHz (fast mode) so reading four sensors doesn't take forever
  Wire.begin();
  Wire.setClock(400000);

  // Gets set to false if anything fails, so one missing sensor doesn't hide behind another
  bool allGood = true;

  // IMU: +-16 g and +-2000 deg/s are the biggest ranges, since a rocket launch is violent
  if (!imu.begin_I2C()) {
    Serial.println("IMU not found");
    allGood = false;
  } else {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_416_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_416_HZ);
  }

  // High-g accelerometer: measures up to 200 g, so it never maxes out during the burn
  if (!highG.begin()) {
    Serial.println("High-g accelerometer not found");
    allGood = false;
  }

  // Barometer: oversampling and the IIR filter average out noise, at the cost of a slower response
  if (!baro.begin_I2C()) {
    Serial.println("Barometer not found");
    allGood = false;
  } else {
    baro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    baro.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    baro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    baro.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // GPS: 10 updates per second, and "autoPVT" means it sends position data on its own so we never wait for it
  if (!gps.begin()) {
    Serial.println("GPS not found");
    allGood = false;
  } else {
    gps.setI2COutput(COMMS_TYPE_UBX);
    gps.setNavigationFrequency(10);
    gps.setAutoPVT(true);
  }

  return allGood;
}

void sensorsZeroBaro() {

  // Average 50 readings (about 1 second) so one noisy reading doesn't set the wrong "zero"
  float sum = 0.0f;
  int count = 0;

  for (int i = 0; i < 50; i++) {
    if (baro.performReading()) {
      sum = sum + baro.pressure;
      count = count + 1;
    }
    delay(20);
  }

  // Only trust the average if we actually got readings, otherwise keep the old value
  if (count > 0) {
    groundPressure = sum / count;
  }
}

void sensorsRead(SensorData &data) {

  // IMU: one call fills in acceleration, rotation, and temperature (we ignore its temperature)
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  data.ax = accel.acceleration.x;
  data.ay = accel.acceleration.y;
  data.az = accel.acceleration.z;

  data.gx = gyro.gyro.x;
  data.gy = gyro.gyro.y;
  data.gz = gyro.gyro.z;

  // High-g accelerometer
  sensors_event_t hg;
  highG.getEvent(&hg);

  data.hx = hg.acceleration.x;
  data.hy = hg.acceleration.y;
  data.hz = hg.acceleration.z;

  // Barometer: performReading() waits for a fresh reading and returns false if it failed
  data.baroValid = baro.performReading();

  if (data.baroValid) {
    data.pressure = baro.pressure;
    data.temperature = baro.temperature;

    // Standard formula that turns pressure into height: lower pressure than the pad means higher up
    data.baroAlt = 44330.0f * (1.0f - powf(baro.pressure / groundPressure, 0.1903f));
  }

  // GPS: getPVT() is true only when a new reading arrived, so otherwise we keep the old values
  if (gps.getPVT()) {

    // 3 means a full 3D fix (latitude, longitude, and altitude)
    data.gpsFix = (gps.getFixType() >= 3);

    // The library gives latitude and longitude as whole numbers in 1e-7 degrees, so we scale them
    data.lat = gps.getLatitude() * 1e-7;
    data.lon = gps.getLongitude() * 1e-7;

    // Altitude comes in millimeters, so divide by 1000 to get meters
    data.gpsAlt = gps.getAltitude() / 1000.0f;
    data.satellites = gps.getSIV();
  }
}
