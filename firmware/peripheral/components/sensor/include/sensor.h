#pragma once

#include "driver/adc.h"
#include "driver/touch_pad.h"
#include "esp_adc_cal.h"

// ------------------------------------------------------
// I2C
// ------------------------------------------------------

#define I2C_MASTER_SCL_IO 21        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 22        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 10000    /*!< I2C master clock frequency (no external pull up, so we need to slow 🐌) */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

esp_err_t i2c_controller_init();
esp_err_t i2c_reg_read_memory(uint8_t reg_addr, uint8_t *data, size_t len, uint8_t delay);
esp_err_t i2c_reg_write(uint8_t reg_addr, uint8_t *data, size_t len);

// ------------------------------------------------------
// Battery
// ------------------------------------------------------

#define ADC_ATTEN ADC_ATTEN_DB_0
#define ADC_CHANNEL ADC_CHANNEL_5
#define ADC_DEFAULT_VREF 1100
#define ADC_NO_OF_SAMPLES 64 // Multisampling
#define ADC_UNIT ADC_UNIT_1
#define ADC_WIDTH ADC_WIDTH_BIT_12

/* pre-calculated value for the voltage divider: 249,000 / (1,500,000 + 249,000) */
#define ADC_VOLT_DIV 249000 / (1500000 + 249000)

esp_adc_cal_characteristics_t *adc_chars;

void adc_init(void);
uint32_t get_voltage();

// ------------------------------------------------------
// HDC1080
// ------------------------------------------------------

#define HDC1080_ADDRESS 0x40

/* HDC1080 register addresses */
#define HDC1080_REG_TEMP 0x00
#define HDC1080_REG_HUMD 0x01
#define HDC1080_REG_CONF 0x02
#define HDC1080_REG_MAN_ID 0xFE
#define HDC1080_REG_DEV_ID 0xFF

/* HDC1080 config values */
#define HDC1080_CONF_RST 1        /* Software Reset */
#define HDC1080_CONF_HEAT_EN 0    /* Heater Disabled */
#define HDC1080_CONF_HEAT_DIS 1   /* Heater Enabled */
#define HDC1080_CONF_MODE_0 0     /* Temperature or Humidity is acquired. */
#define HDC1080_CONF_MODE_1 1     /* Temperature and Humidity are acquired in sequence, Temperature first. */
#define HDC1080_CONF_TRES_14BIT 0 /* Temperature measurement resolution 14 bit */
#define HDC1080_CONF_TRES_11BIT 1 /* Temperature measurement resolution 11 bit */
#define HDC1080_CONF_HRES_14BIT 0 /* Humidity measurement resolution 14 bit */
#define HDC1080_CONF_HRES_11BIT 1 /* Humidity measurement resolution 11 bit */
#define HDC1080_CONF_HRES_8BIT 2  /* Humidity measurement resolution 8 bit */

/* HDC1080 device constants */
#define HDC1080_MAN_ID 0x5449
#define HDC1080_DEV_ID 0x1050

/* I2C config */
#define DELAY_BOTH_MEASUREMENTS 30

typedef union
{
    uint16_t raw;
    struct
    {
        /* Humidity measurement resolution.
         * 0 = 14 bit
         * 1 = 11 bit
         * 2 = 8 bit
         */
        uint8_t humidity_resolution : 2;
        /* Temperature measurement resolution.
         * 0 = 14 bit
         * 1 = 11 bit
         */
        uint8_t temperature_resolution : 1;
        /* Battery status (read only).
         * 0 = voltage > 2.8V
         * 1 = voltage < 2.8V
         */
        uint8_t battery_status : 1;
        /* Mode of acquisition.
         * 0 = temperature or humidity is acquired
         * 1 = temperature and humidity are acquired in sequence, temperature first
         */
        uint8_t mode : 1;
        /* Heater, can be used to test the sensor or to drive condensation off the sensor.
         * 0 = disabled
         * 1 = enabled
         */
        uint8_t heater : 1;
        uint8_t reserved0 : 1;
        /* Software reset.
         * 0 = normal operation
         * 1 = software reset
         */
        uint8_t reset : 1;
        uint8_t reserved1;
    };
} hdc1080_config_t;

typedef struct
{
    /* Temperature in degrees celsius */
    float temperature;
    /* Relative humidity */
    float humidity;
    /* Config of the device */
    hdc1080_config_t config;
} hdc1080_device_t;

hdc1080_device_t *hdc1080_device;

/**
 * Initialize a HDC1080 temperature and humidity sensor device.
 *
 * @param[in] config The configuration to initialize the device with.
 * @return device.
 */
hdc1080_device_t *hdc1080_init(hdc1080_config_t config);

/**
 * Read temperature and humidity at the same time if device is in mode 1 (HDC1080_CONF_MODE_1).
 * The results are stored in the given device.
 *
 * @param[out] device The intialized device.
 * @return esp error.
 */
esp_err_t hdc1080_read_sensor(hdc1080_device_t *device);
esp_err_t hdc1080_read_temperature(hdc1080_device_t *device);
esp_err_t hdc1080_read_humidity(hdc1080_device_t *device);
esp_err_t hdc1080_median_temperature(hdc1080_device_t *device, uint8_t repeats);
esp_err_t hdc1080_median_humidity(hdc1080_device_t *device, uint8_t repeats);

esp_err_t _hdc1080_reg_read_memory(uint8_t reg_addr, uint8_t *data, size_t len, uint8_t delay);
esp_err_t _hdc1080_reg_write(uint8_t reg_addr, uint8_t *data, size_t len);
float _convert_temperature(uint16_t temperature_raw);
float _convert_humidity(uint16_t temperature_raw);
int _compare_floats(const void *a, const void *b);

// ------------------------------------------------------
// SHT40
// ------------------------------------------------------

typedef struct
{
    /* Temperature in degrees celsius */
    float temperature;
    /* Relative humidity */
    float humidity;
} sht40_device_t;

// ------------------------------------------------------
// Moisture
// ------------------------------------------------------

#define M_SENSOR_1_PIN TOUCH_PAD_NUM0
#define M_SENSOR_2_PIN TOUCH_PAD_NUM4
#define M_SENSOR_3_PIN TOUCH_PAD_NUM5

#define M_SENSOR_1_CAL_WATER 140
#define M_SENSOR_2_CAL_WATER 140
#define M_SENSOR_3_CAL_WATER 130
#define M_SENSOR_1_CAL_AIR 2900
#define M_SENSOR_2_CAL_AIR 3100
#define M_SENSOR_3_CAL_AIR 2600

#define M_TOUCH_THRESH_NO_USE 0
#define M_TOUCH_HREF TOUCH_HVOLT_2V4
#define M_TOUCH_LREF TOUCH_LVOLT_0V5
#define M_TOUCH_ATTEN TOUCH_HVOLT_ATTEN_1V5
#define M_TOUCH_SLOPE TOUCH_PAD_SLOPE_7
#define M_TOUCH_TIE TOUCH_PAD_TIE_OPT_HIGH

typedef struct
{
    uint16_t sensor_1_raw;
    uint16_t sensor_2_raw;
    uint16_t sensor_3_raw;
    float sensor_1_cal;
    float sensor_2_cal;
    float sensor_3_cal;
} moisture_t;

moisture_t *moisture_sensor;

moisture_t *moisture_init();
void moisture_deinit(moisture_t *sensor);
void moisture_read_sensor(moisture_t *sensor);