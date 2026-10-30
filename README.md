# Pigeon 9

A reusable autonomous rocket. This repo holds the flight computer firmware and the post-flight analysis tooling.

## Flight computer

Runs on a Teensy 4.1 in C++. It reads an IMU, a high-g accelerometer, a barometer, and GPS, and fuses them with a Kalman filter to estimate altitude and vertical velocity. A flight state machine uses that estimate to detect launch, burnout, apogee, and landing, and fires the recovery system with backup timer and altitude failsafes. Everything gets logged to SD, and live telemetry goes out over LoRa.

## Post-flight analysis

Python scripts that replay logged flights, tune the Kalman filter against real data, and compare results to OpenRocket predictions.

## Layout

- `firmware/` flight computer code (C++)
- `analysis/` log replay and filter tuning (Python)
