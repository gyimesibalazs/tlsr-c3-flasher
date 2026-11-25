// Minimal Arduino / FreeRTOS surface for building bk72xx.cpp on the PC (see bk_sim.cpp). Time is simulated.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
using std::min; using std::max;
enum { INPUT = 1, OUTPUT = 3, INPUT_PULLUP = 5 };
void pinMode(int pin, int mode);
void delay(uint32_t ms);
uint32_t millis();
uint32_t micros();
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
