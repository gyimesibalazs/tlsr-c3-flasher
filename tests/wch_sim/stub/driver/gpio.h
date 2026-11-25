#pragma once
typedef int gpio_num_t;
void gpio_pullup_en(gpio_num_t pin);
int gpio_get_level(gpio_num_t pin);
