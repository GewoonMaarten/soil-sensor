#pragma once

#define GPIO_LED_IO 19
#define GPIO_LED_PIN_SEL 1ULL << GPIO_LED_IO;

void led_init();
void led_toggle();