// sensors.h
// The "menu" for the sensors module: what it offers to the rest of the code.
// Other files #include this header, and the real code lives in sensors.cpp.

// These two lines (and #endif at the bottom) stop the header from being included twice by accident
#ifndef SENSORS_H
#define SENSORS_H

// One snapshot of everything the sensors can tell us
struct SensorData {

  // IMU acceleration in m/s^2 along the rocket's own axes (gravity is still in here)
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;

  // IMU rotation speed in rad/s
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;

  // High-g accelerometer in m/s^2 (the IMU maxes out during the motor burn, this one doesn't)
  float hx = 0.0f;
  float hy = 0.0f;
  float hz = 0.0f;

  // Barometer: pressure in Pa, temperature in C, and altitude in meters ABOVE THE PAD
  float pressure = 0.0f;
  float temperature = 0.0f;
  float baroAlt = 0.0f;

  // False if the barometer failed to give a reading this loop
  bool baroValid = false;

  // GPS: only updates when the GPS has something new, otherwise keeps the last values
  bool gpsFix = false;
  double lat = 0.0;
  double lon = 0.0;
  float gpsAlt = 0.0f;
  int satellites = 0;
};

// Starts every sensor and sets their ranges. Returns false if any sensor is missing
bool sensorsBegin();

// Call this while sitting still on the pad: remembers the ground pressure so altitude starts at 0
void sensorsZeroBaro();

// Fills in "data" with the newest readings from every sensor (the & lets the function change your copy)
void sensorsRead(SensorData &data);

#endif
