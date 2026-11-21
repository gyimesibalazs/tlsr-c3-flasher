#pragma once
#include <stdint.h>
#include <string.h>
#include <algorithm>
using std::min; using std::max;
enum { INPUT = 1, OUTPUT = 3, INPUT_PULLUP = 5, OUTPUT_OPEN_DRAIN = 0x13, PULLUP = 4 };
void pinMode(int pin, int mode);
void delay(uint32_t ms);
uint32_t millis();
