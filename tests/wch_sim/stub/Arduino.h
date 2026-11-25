// Minimal Arduino / FreeRTOS surface for building wch.cpp and wchflash.cpp on the PC (see wch_sim.cpp).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
using std::min; using std::max;
#define IRAM_ATTR
enum { INPUT = 1, OUTPUT = 3, INPUT_PULLUP = 5 };
void pinMode(int pin, int mode);
void delay(uint32_t ms);
uint32_t millis();
uint32_t micros();

typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdMS_TO_TICKS(ms) (ms)
typedef struct SimQueue *QueueHandle_t;
QueueHandle_t xQueueCreate(int len, size_t item);
void xQueueReset(QueueHandle_t q);
BaseType_t xQueueReceive(QueueHandle_t q, void *item, uint32_t ticks);
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *woken);
