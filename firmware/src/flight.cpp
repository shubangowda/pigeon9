// flight.cpp
// The flight state machine (Pad, Armed, Boost, Coast, Descent, Landed) and the event detection for launch, apogee, and landing.
// It also decides when to fire the chute and enforces the safety rules: arming required, never fire on the pad, and a backup timer.
// It takes numbers in and decides what state we are in and whether to fire, with no hardware calls so it can be tested on a laptop.

#include <math.h>
#include "flight.h"

// Helper: true once "cond" has been true continuously for holdMs
// (the flag and the start time are remembered between calls, and the & means we change the caller's copy)
bool FlightComputer::heldFor(bool cond, bool &flag, uint32_t &since, uint32_t now, uint32_t holdMs) {

  // Condition is false: forget any timing we had started
  if (!cond) {
    flag = false;
    return false;
  }

  // Condition just became true: start the clock
  if (!flag) {
    flag = true;
    since = now;
  }

  // True for long enough yet?
  return (now - since) >= holdMs;
}

// Helper: fires the chute (only the first call ever does anything)
void FlightComputer::fire(uint32_t now, uint8_t eventBit) {

  // Once fired, never fire again
  if (pyroFired) {
    return;
  }

  pyroFired = true;
  pyroOn = true;
  pyroStartMs = now;
  deployMs = now;
  events = events | eventBit;
  state = STATE_DESCENT;

  // Start the landing detector from the current altitude
  stillRefAlt = smoothAlt;
  stillSince = now;
}

// Runs once per loop with the latest numbers and decides everything
void FlightComputer::update(const FlightInput &in) {

  uint32_t now = in.nowMs;

  // Smooth the baro altitude so noise doesn't look like a change in height
  if (in.baroValid) {
    if (!smoothReady) {
      smoothAlt = in.baroAlt;
      smoothReady = true;
    } else {

      // Keep dt sane even if the loop stalled for a moment
      float dt = in.dt;
      if (dt < 0.0f) dt = 0.0f;
      if (dt > 0.1f) dt = 0.1f;

      // Move a fraction of the way toward the new reading
      smoothAlt = smoothAlt + (in.baroAlt - smoothAlt) * (dt / (BARO_SMOOTH_TAU_S + dt));
    }
  }

  // Switch the e-match current off after PYRO_FIRE_MS
  if (pyroOn && (now - pyroStartMs) >= PYRO_FIRE_MS) {
    pyroOn = false;
  }

  switch (state) {

    // ---- PAD: sitting there, can never fire ----
    case STATE_PAD:

      // Arm only if the switch is on AND everything started fine AND the baro is working right now
      if (in.armSwitchOn && in.sensorsOk && in.baroValid) {
        state = STATE_ARMED;
        accelHighFlag = false;
        altHighFlag = false;
      }
      break;

    // ---- ARMED: waiting for launch ----
    case STATE_ARMED: {

      // Switch turned off before launch: go back to safe
      if (!in.armSwitchOn) {
        state = STATE_PAD;
        break;
      }

      // Launch check 1: hard acceleration for long enough (total, so it doesn't matter which way the rocket is tilted)
      bool accelLaunch = heldFor(in.accelTotal > LAUNCH_ACCEL_G * GRAVITY, accelHighFlag, accelHighSince, now, LAUNCH_HOLD_MS);

      // Launch check 2 (backup if the accelerometer dies): the baro says we are way above the pad
      bool altLaunch = heldFor(in.baroValid && in.baroAlt > LAUNCH_ALT_M, altHighFlag, altHighSince, now, LAUNCH_ALT_HOLD_MS);

      if (accelLaunch || altLaunch) {
        state = STATE_BOOST;
        launchMs = now;
        launchConfirmed = false;
        events = events | EVENT_LAUNCH;

        // Start tracking the highest altitude and clear the detectors from any earlier activity
        maxSmoothAlt = smoothAlt;
        dropFlag = false;
        kalmanFlag = false;
        confirmAccelFlag = false;
      }
      break;
    }

    // ---- BOOST: motor burning ----
    case STATE_BOOST:

      // Track the highest altitude
      if (in.baroValid && smoothAlt > maxSmoothAlt) {
        maxSmoothAlt = smoothAlt;
      }

      // A real launch is confirmed by height (baro) or by the motor pushing hard for a long time (works even if the baro died)
      if (!launchConfirmed) {
        bool accelConfirm = heldFor(in.accelTotal > LAUNCH_CONFIRM_ACCEL_G * GRAVITY, confirmAccelFlag, confirmAccelSince, now, LAUNCH_CONFIRM_ACCEL_HOLD_MS);

        if ((in.baroValid && in.baroAlt > LAUNCH_CONFIRM_ALT_M) || accelConfirm) {
          launchConfirmed = true;

          // If an earlier bump started the launch clock too soon, move it forward to when the real liftoff must have been
          if (now > LAUNCH_CONFIRM_LAG_MS && (now - LAUNCH_CONFIRM_LAG_MS) > launchMs) {
            launchMs = now - LAUNCH_CONFIRM_LAG_MS;
          }
        } else if ((now - launchMs) >= LAUNCH_CONFIRM_MS) {

          // Never climbed: it was a bump or a glitch, so go back to waiting
          state = STATE_ARMED;
          events = events | EVENT_FALSE_ALARM;
          accelHighFlag = false;
          altHighFlag = false;
          break;
        }
      }

      // Burn is over, start looking for apogee
      if (launchConfirmed && (now - launchMs) >= BOOST_END_MS) {
        state = STATE_COAST;
      }
      break;

    // ---- COAST: going up with no thrust ----
    case STATE_COAST: {

      // Track the highest altitude
      if (in.baroValid && smoothAlt > maxSmoothAlt) {
        maxSmoothAlt = smoothAlt;
      }

      // Apogee detection stays off until the lockout is over
      if ((now - launchMs) >= APOGEE_LOCKOUT_MS) {

        // Kalman apogee: speed has been zero or negative for a moment (this is only LOGGED, it never fires the chute)
        if (!(events & EVENT_KALMAN_APOGEE)) {
          bool kalmanDown = heldFor(in.filtVel <= 0.0f && in.filtAlt > MIN_DEPLOY_ALT_M, kalmanFlag, kalmanSince, now, APOGEE_HOLD_MS);
          if (kalmanDown) {
            events = events | EVENT_KALMAN_APOGEE;
            kalmanApogeeMs = now;
          }
        }

        // Baro apogee: this one fires the chute
        // The smoothed altitude fell below its highest point by APOGEE_DROP_M and stayed there, and we are high enough
        bool baroDown = heldFor(in.baroValid && maxSmoothAlt > MIN_DEPLOY_ALT_M && smoothAlt <= maxSmoothAlt - APOGEE_DROP_M, dropFlag, dropSince, now, APOGEE_HOLD_MS);
        if (baroDown) {
          fire(now, EVENT_BARO_FIRE);
        }
      }

      // Backup timer: fires no matter what the sensors say (only reachable after a confirmed launch)
      if ((now - launchMs) >= BACKUP_TIMER_MS) {
        fire(now, EVENT_TIMER_FIRE);
      }
      break;
    }

    // ---- DESCENT: chute is out, coming down ----
    case STATE_DESCENT:

      // If the altitude moves, restart the "are we still" clock from the new altitude
      if (fabsf(smoothAlt - stillRefAlt) > LANDED_STILL_M) {
        stillRefAlt = smoothAlt;
        stillSince = now;
      }

      // Still for long enough, and well after the chute fired, means we landed
      if ((now - stillSince) >= LANDED_HOLD_MS && (now - deployMs) >= LANDED_HOLD_MS) {
        state = STATE_LANDED;
        events = events | EVENT_LANDED;
      }
      break;

    // ---- LANDED: nothing left to do ----
    case STATE_LANDED:
      break;
  }
}

// Where we are in the flight
FlightState FlightComputer::getState() {
  return state;
}

// True while current should be flowing to the e-match
bool FlightComputer::pyroActive() {
  return pyroOn;
}

// All the event flags so far
uint8_t FlightComputer::getEvents() {
  return events;
}

// Times for the log and for comparing detectors afterward
uint32_t FlightComputer::getLaunchMs() {
  return launchMs;
}

uint32_t FlightComputer::getDeployMs() {
  return deployMs;
}

uint32_t FlightComputer::getKalmanApogeeMs() {
  return kalmanApogeeMs;
}

// True when the buzzer should be beeping so we can find the rocket
// (once landed, or if we are still "descending" a minute after the chute fired, like stuck in a tree)
bool FlightComputer::beaconWanted(uint32_t nowMs) {
  if (state == STATE_LANDED) {
    return true;
  }
  return (state == STATE_DESCENT) && ((nowMs - deployMs) > 60000);
}

// Turns the IMU reading along the rocket's long axis into vertical acceleration for the filter
float flightVerticalAccel(float upAxisAccel) {
  return upAxisAccel - GRAVITY;
}
