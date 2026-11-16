// logging.cpp
// Writes flight data to the SD card at a high rate, including raw sensor readings, filter output, GPS track, state changes, and pyro status.
// Every line is one row of comma separated numbers, and the first line of the file names each column.
// The SD log is what gets replayed in Python after each flight.

// Built-in Arduino stuff and the SD card library that comes with the Teensy
#include <Arduino.h>
#include <SD.h>
#include <stdio.h>
#include "logging.h"

// The first line of every log file: the name of each column, in the same order as the rows below
static const char *HEADER =
  "t_ms,state,pyro,arm,ax,ay,az,gx,gy,gz,hx,hy,hz,press,temp,baro_alt,baro_ok,"
  "gps_fix,lat,lon,gps_alt,sats,filt_alt,filt_vel,vert_accel,events";

// The open log file, and the last time we forced the data onto the card
static File logFile;
static uint32_t lastFlushMs = 0;

bool loggingBegin() {

  // Start the SD slot built into the Teensy 4.1
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card not found");
    return false;
  }

  // Find the first file name that doesn't exist yet so we never overwrite an old flight
  char name[16];
  bool opened = false;

  for (int n = 0; n < 1000; n++) {
    snprintf(name, sizeof(name), "flt%03d.csv", n);

    if (!SD.exists(name)) {
      logFile = SD.open(name, FILE_WRITE);
      opened = true;
      break;
    }
  }

  // All 1000 names used, or the file would not open
  if (!opened || !logFile) {
    Serial.println("Could not open a log file");
    return false;
  }

  // Write the column names and save them right away
  logFile.println(HEADER);
  logFile.flush();

  Serial.print("Logging to ");
  Serial.println(name);
  return true;
}

int loggingFormatRow(char *buf, size_t size, const LogRow &row) {

  // snprintf builds the line: each % is a slot filled by the matching value at the end (never writes past "size")
  // %lu is a whole number, %.2f means a decimal with 2 digits after the point, and so on
  return snprintf(buf, size,
    "%lu,%d,%d,%d,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%d,%d,%.7f,%.7f,%.1f,%d,%.2f,%.2f,%.2f,%u\n",
    (unsigned long)row.tMs, row.state, row.pyro ? 1 : 0, row.arm ? 1 : 0,
    row.s.ax, row.s.ay, row.s.az,
    row.s.gx, row.s.gy, row.s.gz,
    row.s.hx, row.s.hy, row.s.hz,
    row.s.pressure, row.s.temperature, row.s.baroAlt, row.s.baroValid ? 1 : 0,
    row.s.gpsFix ? 1 : 0, row.s.lat, row.s.lon, row.s.gpsAlt, row.s.satellites,
    row.filtAlt, row.filtVel, row.vertAccel,
    (unsigned)row.events);
}

void loggingWriteRow(const LogRow &row) {

  // Build the line in a buffer big enough for the longest possible row
  char line[400];
  int len = loggingFormatRow(line, sizeof(line), row);

  // Skip anything that didn't format properly instead of writing garbage
  if (len <= 0 || len >= (int)sizeof(line)) {
    return;
  }

  logFile.write((const uint8_t *)line, len);

  // Force the data onto the card once a second, so a crash or power loss loses at most about a second
  if ((row.tMs - lastFlushMs) >= 1000) {
    logFile.flush();
    lastFlushMs = row.tMs;
  }
}
