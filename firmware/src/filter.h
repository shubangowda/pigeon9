// filter.h
// The "menu" for the Kalman filter: the class and what other files are allowed to call on it.
// The actual math lives in filter.cpp.

// Stops the header from being included twice by accident
#ifndef FILTER_H
#define FILTER_H

class FlightKalmanFilter {
private:

// What the filter currently thinks: altitude (m) and vertical speed (m/s)
float alt = 0.0f;
float vel = 0.0f;

// How confident the filter is in alt and vel (updates itself every loop)
float P00 = 10.0f;
float P01 = 0.0f;
float P10 = 0.0f;
float P11 = 10.0f;

// How much to distrust our own prediction each loop
float Q_alt = 0.01f;
float Q_vel = 0.1f;

// How noisy the barometer is
float R = 4.0f;

public:

// Runs once per loop: takes the latest acceleration, baro altitude, and time step, and updates the filter
void update(float accel, float baroAlt, float dt);

// Lets other files read the filter's current altitude estimate (m)
float getAlt();

// Lets other files read the filter's current vertical speed estimate (m/s)
float getVel();

// Call this while sitting on the pad: sets altitude and speed to 0 and goes back to being unsure
void reset();
};

#endif
