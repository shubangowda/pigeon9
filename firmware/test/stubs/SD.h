// Fake SD.h so logging.cpp can compile on a laptop for testing (the real one comes with the Teensy)
#ifndef FAKE_SD_H
#define FAKE_SD_H
#include <cstdint>
#include <cstddef>
#define BUILTIN_SDCARD 254
#define FILE_WRITE 1
struct File {
  operator bool() const { return true; }
  void println(const char *) {}
  void flush() {}
  size_t write(const uint8_t *, size_t n) { return n; }
};
struct FakeSD {
  bool begin(int) { return true; }
  bool exists(const char *) { return false; }
  File open(const char *, int) { return File(); }
};
static FakeSD SD;
#endif
