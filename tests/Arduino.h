#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <string.h>

typedef uint8_t byte;

unsigned long micros();

inline void noInterrupts() {}
inline void interrupts() {}

#endif
