// main.cpp
// Entry point for the Pigeon 9 flight computer. setup() initializes every module once, and loop() runs forever,
// repeating: read sensors, run the filter, run the flight state machine, then log the data.
// This file should stay tiny and just call into the other modules, and it is the only place that touches the pyro pin.

#include <Arduino.h>
#include <math.h>
#include "sensors.h"
#include "filter.h"
#include "flight.h"
#include "logging.h"

// ==================== PINS: CHANGE THESE TO MATCH YOUR WIRING ====================

// Arming switch between this pin and GND (switch closed = ARMED)
const int ARM_SWITCH_PIN = 2;

// Gate of the MOSFET that sends current to the e-match (needs a 10k resistor from gate to GND so it can never float on)
const int PYRO_PIN = 3;

// Active buzzer (the kind that beeps by itself when powered)
const int BUZZER_PIN = 4;

// Built-in LED
const int LED_PIN = 13;

// ==================== SETTINGS ====================

// Run the loop every 10 ms (100 times a second), and never faster because the baro needs about 7 ms per reading
const uint32_t LOOP_US = 10000;

// The rocket must be nearly upright to arm: sitting still on the pad the up-axis reads 1 g, so it must be within this of gravity
const float UPRIGHT_TOLERANCE = 2.0f;

// The switch must hold steady this long before we believe it
const uint32_t DEBOUNCE_MS = 50;

// ==================== THINGS THE LOOP REMEMBERS ====================

FlightKalmanFilter filter;
FlightComputer flight;
SensorData sensors;

// True only if every sensor and the SD card started
bool systemOk = false;
bool logOk = false;

// Last good baro altitude, so one failed reading doesn't feed garbage to the filter
float lastBaroAlt = 0.0f;

// Loop timing
uint32_t lastLoopUs = 0;

// Arming switch: raw reading, when it last changed, the debounced value, and the previous debounced value
bool armRaw = false;
uint32_t armChangeMs = 0;
bool armStable = false;
bool armPrev = false;

// Becomes true once we have seen the switch OFF since boot (so powering on with the switch already ON can never arm)
bool armSeenOff = false;

// Once-a-second status printing for bench testing
uint32_t lastPrintMs = 0;
uint32_t loopCount = 0;

// Beeps a pattern on the buzzer: 0 = silent, 1 = armed chirp, 2 = error, 3 = beacon
void updateBuzzer(uint32_t nowMs, int mode) {
  bool on = false;

  // 100 ms chirp every second means armed
  if (mode == 1) {
    on = (nowMs % 1000) < 100;
  }

  // Fast beeping means something is wrong
  if (mode == 2) {
    on = (nowMs % 400) < 200;
  }

  // Long slow beeps means come find the rocket
  if (mode == 3) {
    on = (nowMs % 1000) < 500;
  }

  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
}

void setup() {

  // FIRST THING: make sure the pyro pin is off before anything else can go wrong
  pinMode(PYRO_PIN, OUTPUT);
  digitalWrite(PYRO_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);

  // INPUT_PULLUP keeps the pin high until the switch connects it to GND
  pinMode(ARM_SWITCH_PIN, INPUT_PULLUP);

  // Start USB serial for bench testing (we do NOT wait for a computer, or the rocket would hang forever on the pad)
  Serial.begin(115200);

  // Give the sensors a moment to power up
  delay(1000);

  // Read where the switch is right now, so a switch that is already ON at power-up is never mistaken for "off"
  armRaw = (digitalRead(ARM_SWITCH_PIN) == LOW);
  armStable = armRaw;
  armPrev = armRaw;
  armChangeMs = millis();
  armSeenOff = !armRaw;

  // Start everything and remember if anything failed
  bool sensorsOk = sensorsBegin();
  logOk = loggingBegin();
  systemOk = sensorsOk && logOk;

  if (systemOk) {

    // Set altitude 0 to wherever we are sitting right now
    sensorsZeroBaro();
    filter.reset();

    // One chirp means all good
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
  }

  lastLoopUs = micros();
}

void loop() {

  // Not time for the next loop yet, so go around again
  uint32_t nowUs = micros();
  if ((nowUs - lastLoopUs) < LOOP_US) {
    return;
  }

  // How long this loop took, and keep it sane even if something stalled for a moment
  float dt = (nowUs - lastLoopUs) / 1000000.0f;
  lastLoopUs = nowUs;
  if (dt < 0.001f) dt = 0.001f;
  if (dt > 0.05f) dt = 0.05f;

  uint32_t nowMs = millis();
  loopCount++;

  // ---- 1. Read every sensor ----
  sensorsRead(sensors);

  // Only use the baro altitude if this reading was good, otherwise keep the last good one
  if (sensors.baroValid) {
    lastBaroAlt = sensors.baroAlt;
  }

  // ---- 2. Run the filter ----
  // sensors.az is the axis pointing up the rocket (change it if the IMU is mounted another way)
  float vertAccel = flightVerticalAccel(sensors.az);
  filter.update(vertAccel, lastBaroAlt, dt);

  // ---- 3. Read the arming switch with debouncing ----
  // The switch is wired to GND, so LOW means closed
  bool raw = (digitalRead(ARM_SWITCH_PIN) == LOW);
  if (raw != armRaw) {
    armRaw = raw;
    armChangeMs = nowMs;
  }
  if ((nowMs - armChangeMs) >= DEBOUNCE_MS) {
    armStable = raw;
  }

  // Seeing the switch off at least once is required before it can ever arm
  if (!armStable) {
    armSeenOff = true;
  }

  // The moment the switch flips on while on the pad: set altitude 0 again (takes about a second, and the rocket is sitting still)
  if (armStable && !armPrev && armSeenOff && flight.getState() == STATE_PAD && systemOk) {
    sensorsZeroBaro();
    filter.reset();
    lastLoopUs = micros();
  }
  armPrev = armStable;

  // ---- 4. Run the flight computer ----
  // On the pad it must also be upright (this catches an IMU mounted on the wrong axis, or the rocket still lying down)
  bool upright = fabsf(sensors.az - GRAVITY) < UPRIGHT_TOLERANCE;
  bool readyToArm = systemOk && (flight.getState() != STATE_PAD || upright);

  FlightInput in;
  in.nowMs = nowMs;
  in.dt = dt;
  in.accelTotal = sqrtf(sensors.ax * sensors.ax + sensors.ay * sensors.ay + sensors.az * sensors.az);
  in.baroAlt = sensors.baroAlt;
  in.baroValid = sensors.baroValid;
  in.filtAlt = filter.getAlt();
  in.filtVel = filter.getVel();
  in.armSwitchOn = armStable && armSeenOff;
  in.sensorsOk = readyToArm;

  flight.update(in);

  // ---- 5. Fire (or don't) right away, before the slow SD card write ----
  digitalWrite(PYRO_PIN, flight.pyroActive() ? HIGH : LOW);

  // ---- 6. Log everything ----
  if (logOk) {
    LogRow row;
    row.tMs = nowMs;
    row.state = (int)flight.getState();
    row.pyro = flight.pyroActive();
    row.arm = armStable;
    row.s = sensors;
    row.filtAlt = in.filtAlt;
    row.filtVel = in.filtVel;
    row.vertAccel = vertAccel;
    row.events = flight.getEvents();
    loggingWriteRow(row);
  }

  // ---- 7. Buzzer and LED so you can tell what it is thinking ----
  int buzzMode = 0;

  // Armed: chirp every second
  if (flight.getState() == STATE_ARMED) {
    buzzMode = 1;
  }

  // Switch is ON but it is still not armed: something is wrong (not upright, bad sensor, or no baro)
  if (armStable && flight.getState() == STATE_PAD) {
    buzzMode = 2;
  }

  // A sensor or the SD card failed at startup
  if (!systemOk) {
    buzzMode = 2;
  }

  // Landed: beacon so we can find it
  if (flight.beaconWanted(nowMs)) {
    buzzMode = 3;
  }

  updateBuzzer(nowMs, buzzMode);

  // LED is on whenever we are not sitting safe on the pad
  digitalWrite(LED_PIN, flight.getState() != STATE_PAD ? HIGH : LOW);

  // ---- 8. Bench status once a second (never in flight, USB printing can stall) ----
  if (Serial && flight.getState() <= STATE_ARMED && (nowMs - lastPrintMs) >= 1000) {
    Serial.printf("state=%d loop=%luHz az=%.2f baro=%.2fm filt=%.2fm sats=%d ok=%d\n",
      (int)flight.getState(), (unsigned long)loopCount, sensors.az, sensors.baroAlt, in.filtAlt, sensors.satellites, systemOk ? 1 : 0);
    loopCount = 0;
    lastPrintMs = nowMs;
  }
}
