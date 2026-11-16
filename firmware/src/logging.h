// logging.h
// The "menu" for the logging module: one row of flight data and the functions that save it to the SD card.
// The real code lives in logging.cpp.

// Stops the header from being included twice by accident
#ifndef LOGGING_H
#define LOGGING_H

#include <stdint.h>
#include <stddef.h>
#include "sensors.h"

// Everything we save on one line of the log
struct LogRow {
  uint32_t tMs;        // time since the Teensy booted, in milliseconds
  int state;           // flight state number (0 pad, 1 armed, 2 boost, 3 coast, 4 descent, 5 landed)
  bool pyro;           // true while current is flowing to the e-match
  bool arm;            // the arming switch
  SensorData s;        // every raw sensor reading
  float filtAlt;       // Kalman filter altitude
  float filtVel;       // Kalman filter vertical speed
  float vertAccel;     // the vertical acceleration we fed the filter
  uint8_t events;      // event flags (launch, apogee detections, chute fired, landed)
};

// Starts the SD card and creates a new log file (flt000.csv, flt001.csv, and so on). Returns false if it fails
bool loggingBegin();

// Turns one row into a line of text for the file, and returns how many characters it wrote
// (kept separate from the SD card so it can be tested on a laptop)
int loggingFormatRow(char *buf, size_t size, const LogRow &row);

// Saves one row to the SD card
void loggingWriteRow(const LogRow &row);

#endif
