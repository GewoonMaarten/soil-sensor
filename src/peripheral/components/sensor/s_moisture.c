#include "driver/touch_pad.h"
#include "esp_log.h"

#include "sensor.h"

static const char *TAG = __FILE__;

moisture_t *moisture_init()
{
    moisture_t *device;

    if ((device = malloc(sizeof(moisture_t))) == NULL)
    {
        return NULL;
    }

    touch_pad_init();

    touch_pad_set_voltage(M_TOUCH_HREF, M_TOUCH_LREF, M_TOUCH_ATTEN);

    touch_pad_set_cnt_mode(M_SENSOR_1_PIN, M_TOUCH_SLOPE, M_TOUCH_TIE);
    touch_pad_set_cnt_mode(M_SENSOR_2_PIN, M_TOUCH_SLOPE, M_TOUCH_TIE);
    touch_pad_set_cnt_mode(M_SENSOR_3_PIN, M_TOUCH_SLOPE, M_TOUCH_TIE);

    touch_pad_config(M_SENSOR_1_PIN, M_TOUCH_THRESH_NO_USE);
    touch_pad_config(M_SENSOR_2_PIN, M_TOUCH_THRESH_NO_USE);
    touch_pad_config(M_SENSOR_3_PIN, M_TOUCH_THRESH_NO_USE);

    return device;
}

void moisture_deinit(moisture_t *sensor)
{
    free(sensor);
    touch_pad_deinit();
}

void moisture_read_sensor(moisture_t *sensor)
{
    touch_pad_read(M_SENSOR_1_PIN, &(sensor->sensor_1_raw));
    touch_pad_read(M_SENSOR_2_PIN, &(sensor->sensor_2_raw));
    touch_pad_read(M_SENSOR_3_PIN, &(sensor->sensor_3_raw));

    sensor->sensor_1_cal = (float)(sensor->sensor_1_raw - M_SENSOR_1_CAL_WATER) * -100 / (float)(M_SENSOR_1_CAL_AIR - M_SENSOR_1_CAL_WATER) + 100;
    sensor->sensor_2_cal = (float)(sensor->sensor_2_raw - M_SENSOR_2_CAL_WATER) * -100 / (float)(M_SENSOR_2_CAL_AIR - M_SENSOR_2_CAL_WATER) + 100;
    sensor->sensor_3_cal = (float)(sensor->sensor_3_raw - M_SENSOR_3_CAL_WATER) * -100 / (float)(M_SENSOR_3_CAL_AIR - M_SENSOR_3_CAL_WATER) + 100;

    ESP_LOGD(TAG, "Moisture RAW M1: %d, M2: %d, M3: %d", sensor->sensor_1_raw, sensor->sensor_2_raw, sensor->sensor_3_raw);
    ESP_LOGD(TAG, "Moisture CAl M1: %f, M2: %f, M3: %f", sensor->sensor_1_cal, sensor->sensor_2_cal, sensor->sensor_3_cal);
}