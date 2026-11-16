// Fake Arduino.h so logging.cpp can compile on a laptop for testing (the real one comes with the Teensy)
#ifndef FAKE_ARDUINO_H
#define FAKE_ARDUINO_H
#include <cstdint>
#include <cstdio>
struct FakeSerial {
  void print(const char *s) { printf("%s", s); }
  void println(const char *s) { printf("%s\n", s); }
};
static FakeSerial Serial;
#endif
