// flight.h
// The "menu" for the flight computer: its settings, its inputs and outputs, and what other files can ask it.
// The real logic lives in flight.cpp, and it never touches hardware so it can be tested on a laptop.

// Stops the header from being included twice by accident
#ifndef FLIGHT_H
#define FLIGHT_H

#include <stdint.h>

// ==================== SETTINGS: CHECK EVERY ONE OF THESE BEFORE FLYING ====================

// Gravity in m/s^2
const float GRAVITY = 9.81f;

// Launch detection: total acceleration above this many g (1 g = sitting still) for this long means "launched"
const float LAUNCH_ACCEL_G = 3.0f;
const uint32_t LAUNCH_HOLD_MS = 50;

// Backup launch detection in case the accelerometer dies: baro altitude above this for this long means "launched"
const float LAUNCH_ALT_M = 20.0f;
const uint32_t LAUNCH_ALT_HOLD_MS = 100;

// A real launch must be confirmed within LAUNCH_CONFIRM_MS, otherwise it was a bump on the pad and we go back to ARMED
// Confirmed means either the baro says we climbed this high, or the motor kept pushing hard for a long time (a bump never does)
const float LAUNCH_CONFIRM_ALT_M = 10.0f;
const float LAUNCH_CONFIRM_ACCEL_G = 5.0f;
const uint32_t LAUNCH_CONFIRM_ACCEL_HOLD_MS = 500;
const uint32_t LAUNCH_CONFIRM_MS = 3000;

// Both confirmations take about this long after the real liftoff, so confirming moves the launch clock to "now minus this"
// (otherwise a bump on the pad just before a real launch would start the backup timer too early)
const uint32_t LAUNCH_CONFIRM_LAG_MS = 500;

// How long after launch we call it "boost" before switching to "coast" (the G80 burns about 1.2 s)
const uint32_t BOOST_END_MS = 1500;

// No apogee detection of any kind until this long after launch, so the motor burn can never fool it
const uint32_t APOGEE_LOCKOUT_MS = 3000;

// The baro apogee detector will not fire the chute below this height above the pad
const float MIN_DEPLOY_ALT_M = 30.0f;

// Apogee = the smoothed baro altitude has dropped this far below its highest point, and stayed there this long
const float APOGEE_DROP_M = 3.0f;
const uint32_t APOGEE_HOLD_MS = 100;

// How hard to smooth the baro altitude (bigger = smoother but slower to react)
const float BARO_SMOOTH_TAU_S = 0.1f;

// BACKUP TIMER: fires the chute this long after launch no matter what, in case the baro fails in flight
// SET THIS FROM OPENROCKET: time to apogee plus about 1.5 seconds
// (if it is SHORTER than the real time to apogee the chute fires while the rocket is still going up, even on a normal flight)
// 10500 is only right for a rocket that peaks at about 9 s, so check your own simulation
const uint32_t BACKUP_TIMER_MS = 10500;

// How long to keep current flowing to the e-match
const uint32_t PYRO_FIRE_MS = 1000;

// Landed = altitude has not changed by more than this much for this long (and at least this long after deploy)
const float LANDED_STILL_M = 4.0f;
const uint32_t LANDED_HOLD_MS = 5000;

// ==================== STATES, INPUTS, AND THE FLIGHT COMPUTER ====================

// Where we are in the flight
enum FlightState {
  STATE_PAD = 0,      // sitting there, switch off, can never fire
  STATE_ARMED = 1,    // switch on, waiting for launch
  STATE_BOOST = 2,    // motor burning
  STATE_COAST = 3,    // going up with no thrust, looking for apogee
  STATE_DESCENT = 4,  // chute fired, coming down
  STATE_LANDED = 5    // not moving anymore
};

// Event flags (bits in one number so the log can hold them all in one column)
const uint8_t EVENT_LAUNCH = 1;          // launch detected
const uint8_t EVENT_KALMAN_APOGEE = 2;   // the Kalman filter thinks we hit apogee (logged only, never fires)
const uint8_t EVENT_BARO_FIRE = 4;       // chute fired because of the baro apogee detector
const uint8_t EVENT_TIMER_FIRE = 8;      // chute fired because of the backup timer
const uint8_t EVENT_FALSE_ALARM = 16;    // a bump looked like a launch but never climbed
const uint8_t EVENT_LANDED = 32;         // landed

// Everything the flight computer needs to know each loop
struct FlightInput {
  uint32_t nowMs;      // time in milliseconds
  float dt;            // seconds since the last loop
  float accelTotal;    // total acceleration from the IMU in m/s^2 (any direction)
  float baroAlt;       // raw baro altitude above the pad in meters
  bool baroValid;      // false if the baro reading failed this loop
  float filtAlt;       // Kalman filter altitude in meters
  float filtVel;       // Kalman filter vertical speed in m/s
  bool armSwitchOn;    // the arming switch
  bool sensorsOk;      // every sensor and the SD card started fine
};

class FlightComputer {
private:

// The current state, and when the launch and the chute firing happened
FlightState state = STATE_PAD;
uint32_t launchMs = 0;
uint32_t deployMs = 0;
bool launchConfirmed = false;

// Timers that check a condition has been true for long enough (a flag plus the time it started)
bool accelHighFlag = false;
uint32_t accelHighSince = 0;
bool altHighFlag = false;
uint32_t altHighSince = 0;
bool dropFlag = false;
uint32_t dropSince = 0;
bool kalmanFlag = false;
uint32_t kalmanSince = 0;
bool confirmAccelFlag = false;
uint32_t confirmAccelSince = 0;

// Smoothed baro altitude and the highest value it has reached
bool smoothReady = false;
float smoothAlt = 0.0f;
float maxSmoothAlt = 0.0f;

// Chute firing: it can only ever happen once
bool pyroFired = false;
bool pyroOn = false;
uint32_t pyroStartMs = 0;

// Landing detection
float stillRefAlt = 0.0f;
uint32_t stillSince = 0;

// Event flags and when the Kalman filter thought apogee was (for comparing against the baro later)
uint8_t events = 0;
uint32_t kalmanApogeeMs = 0;

// Helper: true once "cond" has been true continuously for holdMs
bool heldFor(bool cond, bool &flag, uint32_t &since, uint32_t now, uint32_t holdMs);

// Helper: fires the chute (only the first call ever does anything)
void fire(uint32_t now, uint8_t eventBit);

public:

// Runs once per loop with the latest numbers and decides everything
void update(const FlightInput &in);

// Where we are in the flight
FlightState getState();

// True while current should be flowing to the e-match
bool pyroActive();

// All the event flags so far
uint8_t getEvents();

// Times for the log and for comparing detectors afterward
uint32_t getLaunchMs();
uint32_t getDeployMs();
uint32_t getKalmanApogeeMs();

// True when the buzzer should be beeping so we can find the rocket
bool beaconWanted(uint32_t nowMs);
};

// Turns the IMU reading along the rocket's long axis into vertical acceleration for the filter
// (sitting still it reads +1 g, so subtracting gravity gives 0, and it assumes the rocket points straight up)
float flightVerticalAccel(float upAxisAccel);

#endif
