// filter.cpp
// The Kalman filter that fuses accelerometer and barometer data into an estimate of altitude and vertical velocity.
// It is pure math with no hardware calls, so it can be tested on a laptop with logged flight data.
// The noise values (Q and R) are tuned in Python first, then copied in here.
