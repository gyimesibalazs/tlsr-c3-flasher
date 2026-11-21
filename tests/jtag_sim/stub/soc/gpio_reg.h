#pragma once
#include <stdint.h>
enum { GPIO_OUT_W1TS_REG = 1, GPIO_OUT_W1TC_REG, GPIO_ENABLE_W1TS_REG, GPIO_ENABLE_W1TC_REG, GPIO_IN_REG };
void sim_reg_write(int reg, uint32_t v);
uint32_t sim_reg_read(int reg);
#define REG_WRITE(r, v) sim_reg_write((r), (v))
#define REG_READ(r) sim_reg_read(r)
