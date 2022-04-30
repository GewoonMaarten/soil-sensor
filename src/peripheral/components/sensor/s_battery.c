#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sensor.h"

void adc_init(void)
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT, ADC_ATTEN, ADC_WIDTH, ADC_DEFAULT_VREF, adc_chars);
}

uint32_t get_voltage()
{
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < ADC_NO_OF_SAMPLES; i++)
    {
        adc_reading += adc1_get_raw((adc1_channel_t)ADC_CHANNEL);
    }
    adc_reading /= ADC_NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return voltage;
}