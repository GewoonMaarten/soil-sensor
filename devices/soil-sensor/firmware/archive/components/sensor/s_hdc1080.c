#include "stdlib.h"
#include "math.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "sensor.h"

#define ERR_CHECK(f)                                                                       \
    err = f;                                                                               \
    if (err != ESP_OK)                                                                     \
    {                                                                                      \
        ESP_LOGE(TAG, "Error: %d, __file__: %s, __line__: %d\n", err, __FILE__, __LINE__); \
        goto end;                                                                          \
    }

static const char *TAG = __FILE__;

hdc1080_device_t *hdc1080_init(hdc1080_config_t config)
{
    hdc1080_device_t *device;
    esp_err_t err = ESP_OK;

    if ((device = malloc(sizeof(hdc1080_device_t))) == NULL)
    {
        return NULL;
    }

    ERR_CHECK(i2c_reg_write(HDC1080_REG_CONF, (uint8_t *)&(config.raw), sizeof(config.raw)));
    device->config = config;

end:
    return device;
}

esp_err_t hdc1080_read_sensor(hdc1080_device_t *device)
{
    esp_err_t err = ESP_OK;

    uint8_t data[4];
    uint16_t temperature_raw;
    uint16_t humidity_raw;

    assert(device->config.mode == HDC1080_CONF_MODE_1);

    ERR_CHECK(_hdc1080_reg_read_memory(HDC1080_REG_TEMP, data, 4, DELAY_BOTH_MEASUREMENTS));

    temperature_raw = data[0] << 8 | data[1];
    humidity_raw = data[2] << 8 | data[3];

    device->temperature = _convert_temperature(temperature_raw);
    device->humidity = _convert_humidity(humidity_raw);

    ESP_LOGD(TAG, "Temperature: %f", device->temperature);
    ESP_LOGD(TAG, "Humidity: %f", device->humidity);
end:
    return err;
}

esp_err_t hdc1080_read_temperature(hdc1080_device_t *device)
{
    esp_err_t err = ESP_OK;

    uint8_t delay_time;
    uint8_t data[2];
    uint16_t temperature_raw;
    uint8_t delay_multiplier = 2;

    assert(device->config.mode == HDC1080_CONF_MODE_0);

    // Get delay time. First value is specified in datasheet, but it does not seem enough. Multiply by 3 seems to work.
    switch (device->config.temperature_resolution)
    {
    case HDC1080_CONF_TRES_11BIT:
        delay_time = 3.65 * delay_multiplier;
        break;
    case HDC1080_CONF_TRES_14BIT:
        delay_time = 6.35 * delay_multiplier;
        break;
    default:
        assert(0);
        break;
    }

    ERR_CHECK(i2c_reg_read_memory(HDC1080_REG_TEMP, data, 2, delay_time));
    temperature_raw = data[0] << 8 | data[1];
    device->temperature = _convert_temperature(temperature_raw);

    // ESP_LOGD(TAG, "Temperature raw: 0x%02x%02x", data[0], data[1]);
    ESP_LOGD(TAG, "Temperature: %f", device->temperature);
end:
    return err;
}

esp_err_t hdc1080_read_humidity(hdc1080_device_t *device)
{
    esp_err_t err = ESP_OK;

    uint8_t delay_time;
    uint8_t data[2];
    uint16_t humidity_raw;
    uint8_t delay_multiplier = 2;

    assert(device->config.mode == HDC1080_CONF_MODE_0);

    // Get delay time. First value is specified by the datasheet, but it does not seem enough. Multiply by 3 seems to work.
    switch (device->config.humidity_resolution)
    {
    case HDC1080_CONF_HRES_8BIT:
        delay_time = 2.50 * delay_multiplier;
        break;
    case HDC1080_CONF_HRES_11BIT:
        delay_time = 3.85 * delay_multiplier;
        break;
    case HDC1080_CONF_HRES_14BIT:
        delay_time = 6.50 * delay_multiplier;
        break;
    default:
        assert(0);
        break;
    }

    ERR_CHECK(i2c_reg_read_memory(HDC1080_REG_HUMD, data, 2, delay_time));
    humidity_raw = data[0] << 8 | data[1];
    device->humidity = _convert_humidity(humidity_raw);

    ESP_LOGD(TAG, "Humidity: %f", device->humidity);
end:
    return err;
}

esp_err_t hdc1080_median_temperature(hdc1080_device_t *device, uint8_t repeats)
{
    esp_err_t err = ESP_OK;

    float *measurements = (float *)malloc(sizeof(float) * repeats);
    if (measurements == NULL)
    {
        err = ESP_ERR_NO_MEM;
        return err;
    }

    for (uint8_t i = 0; i < repeats; i++)
    {
        ERR_CHECK(hdc1080_read_temperature(device));
        measurements[i] = device->temperature;
    }

    qsort(measurements, repeats, sizeof(float), _compare_floats);

    // Set median temperature, which is the middle item in a sorted list.
    device->temperature = measurements[(repeats + 1) / 2 - 1];

end:
    free(measurements);
    return err;
}

esp_err_t hdc1080_median_humidity(hdc1080_device_t *device, uint8_t repeats)
{
    esp_err_t err = ESP_OK;

    float *measurements = (float *)malloc(sizeof(float) * repeats);
    if (measurements == NULL)
    {
        err = ESP_ERR_NO_MEM;
        return err;
    }

    for (uint8_t i = 0; i < repeats; i++)
    {
        ERR_CHECK(hdc1080_read_humidity(device));
        measurements[i] = device->humidity;
    }

    qsort(measurements, repeats, sizeof(float), _compare_floats);

    // Set median temperature, which is the middle item in a sorted list.
    device->humidity = measurements[(repeats + 1) / 2 - 1];

end:
    free(measurements);
    return err;
}

float _convert_temperature(uint16_t temperature_raw)
{
    return (temperature_raw / pow(2, 16)) * 165.0 - 40.0;
}

float _convert_humidity(uint16_t humidity_raw)
{
    return (humidity_raw / pow(2, 16)) * 100;
}

int _compare_floats(const void *a, const void *b)
{
    float *x = (float *)a;
    float *y = (float *)b;

    if (*x < *y)
        return -1;
    else if (*x > *y)
        return 1;
    return 0;
}