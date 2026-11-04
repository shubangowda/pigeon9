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

  // Our confidence gets worse every loop because we only guessed (each line uses the old values, so keep this order)
  P00 = P00 + dt * (P10 + P01) + dt * dt * P11 + Q_alt;
  P01 = P01 + dt * P11;
  P10 = P10 + dt * P11;
  P11 = P11 + Q_vel;

  // How far off our guess was from what the barometer says
  float error = baroAlt - alt;

  // How much total doubt there is: our own doubt in altitude plus the baro's noise
  float total_doubt = P00 + R;

  // How much to trust the baro: close to 1 means follow the baro, close to 0 means ignore it
  float trust_alt = P00 / total_doubt;
  float trust_vel = P10 / total_doubt;

  // Nudge the altitude and speed toward the baro by the amount we trust it
  alt = alt + trust_alt * error;
  vel = vel + trust_vel * error;

  // Save the old P values because the next lines need them before they get changed
  float old_P00 = P00;
  float old_P01 = P01;

  // We just got real data, so our confidence gets better
  P00 = old_P00 - trust_alt * old_P00;
  P01 = old_P01 - trust_alt * old_P01;
  P10 = P10 - trust_vel * old_P00;
  P11 = P11 - trust_vel * old_P01;
}

// Lets other files read the filter's current altitude estimate (m)
float getAlt() {
  return alt;
}

// Lets other files read the filter's current vertical speed estimate (m/s)
float getVel() {
  return vel;
}

// Call this while sitting on the pad: sets altitude and speed to 0 and goes back to being unsure
void reset() {
  alt = 0.0f;
  vel = 0.0f;
  P00 = 10.0f;
  P01 = 0.0f;
  P10 = 0.0f;
  P11 = 10.0f;
}
};
