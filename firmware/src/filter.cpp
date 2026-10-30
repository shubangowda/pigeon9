// filter.cpp
// The Kalman filter that fuses accelerometer and barometer data into an estimate of altitude and vertical velocity.
// It is pure math with no hardware calls, so it can be tested on a laptop with logged flight data.
// The noise values (Q and R) are tuned in Python first, then copied in here.

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
void update(float accel, float baroAlt, float dt) {

  // Guess the new altitude: old altitude + distance traveled since the last loop
  alt = alt + vel * dt + 0.5f * accel * dt * dt;

  // Guess the new speed: old speed + how much the acceleration changed it
  vel = vel + accel * dt;
}
};

