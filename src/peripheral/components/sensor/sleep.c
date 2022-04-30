#include "sleep.h"
#include "esp_sleep.h"

#include <stdio.h>

void deep_sleep(uint64_t us)
{
    esp_sleep_enable_timer_wakeup(us);
    esp_deep_sleep_start();
}

void hibernate(uint64_t us)
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);

    deep_sleep(us);
}