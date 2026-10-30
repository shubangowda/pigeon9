# Pigeon 9

A solo-built, reusable, autonomous model rocket. The name is a joke nod to Falcon 9, and pigeons are homing birds, which is the long-term goal: a rocket that flies itself and comes back to where it started.

This repo holds all the flight software, avionics firmware, and post-flight analysis tooling for the project.

**Status: early stage. Hardware is being ordered and the repo is intentionally empty of code for now.**

## The rocket

- Airframe: Madcow 2.6" Discovery (39.5 in long, 2.6 in diameter, 29mm motor mount, single deploy)
- Motor: Cesaroni Skidmark G80 (29mm, 2-grain)
- Modeled in OpenRocket: about 696 g loaded, stability around 2.6 cal, predicted apogee around 426 m
- Recovery: electronic deployment fired by the flight computer, with an independent commercial backup altimeter on its own battery

## The flight computer

Runs on a Teensy 4.1 and is written in C++ (Arduino framework or PlatformIO).

Planned sensors and hardware:

- IMU/gyro (LSM6DSOX class) plus an ADXL375 high-g accelerometer for the boost phase
- Barometer (BMP390 or MS5611)
- u-blox MAX-M10S GPS at 10 Hz
- RFM95W 915 MHz LoRa radios for telemetry to a ground station
- microSD for high-rate logging
- MOSFET-driven pyro channels with continuity checks, plus an arming switch

Planned software:

- Sensor drivers running at a steady rate
- A Kalman filter fusing accelerometer and barometer data to estimate altitude and vertical velocity
- A flight state machine: Pad, Armed, Boost, Coast, Apogee, Descent, Main deploy, Landed
- Event detection with sanity checks (launch, burnout, apogee, landing)
- Pyro control with failsafes: backup apogee timer, altitude-gated deployment, no firing on the pad, arming required
- SD logging of raw sensor data, GPS track, events, and pyro health
- LoRa telemetry (GPS, altitude, state)

## Post-flight analysis

Python tooling for replaying logged flights, tuning the Kalman filter (process and measurement noise) against real data, and comparing actual flights to OpenRocket predictions.

The filter gets prototyped and tuned in Python first, then ported to C++ with the tuned numbers. It runs alongside baro-only detection on early flights and is not trusted to fire the chute until it agrees with the backup altimeter.

## Roadmap

- **Mk1:** Basic autonomous recovery. Apogee detection, chute deployment, full logging, Kalman filter, telemetry.
- **Mk2:** Drag flaps with closed-loop apogee targeting.
- **Mk3:** GPS-guided steerable parafoil return. Prototyped with drone or hill drops first.
- **Later:** GPS drift-compensated deploy (needs a dual deploy airframe), possibly TVC or gyro-stabilized ascent.

## Planned layout

- `firmware/` flight computer code for the Teensy 4.1 (C++)
- `analysis/` Python log replay, filter tuning, and OpenRocket comparison

## Safety

Flights happen from a private field with the landowner's permission, with a large clear area in every direction, away from airports, and under FAA weight limits. Early flights stay low, and every flight gets logged, failures included.
