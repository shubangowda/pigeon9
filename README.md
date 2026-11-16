# Pigeon 9

A reusable autonomous rocket. This repo holds the flight computer firmware and the post-flight analysis tooling.

## Flight computer

Runs on a Teensy 4.1 in C++. It reads an IMU, a high-g accelerometer, a barometer, and GPS, and runs a Kalman filter to estimate altitude and vertical velocity. A flight state machine detects launch, apogee, and landing, and fires the chute from a barometer apogee detector with a backup timer. The Kalman filter's apogee call is logged for comparison until it has been proven on real flights. Everything gets logged to SD.

## Post-flight analysis

Python scripts that replay logged flights, tune the Kalman filter against real data, and compare results to OpenRocket predictions.

## Layout

- `firmware/src/` flight computer code (C++)
- `firmware/test/` laptop flight simulation that tests the flight logic
- `analysis/` log replay and filter tuning (Python)
