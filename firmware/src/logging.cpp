// logging.cpp
// Writes flight data to the SD card at a high rate, including raw sensor readings, filter output, GPS track, state changes, and pyro health.
// It also sends live telemetry (GPS, altitude, flight state) to the ground station over the LoRa radio.
// The SD log is what gets replayed in Python after each flight.
