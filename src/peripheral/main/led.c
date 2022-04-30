#include "driver/gpio.h"

#include "led.h"

void led_init()
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

void led_toggle()
{
    gpio_set_level(GPIO_LED_IO, !gpio_get_level(GPIO_LED_IO));
}
