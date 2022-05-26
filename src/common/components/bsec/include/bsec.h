#pragma once

#include <bme68x.h>
#include <drivers/i2c.h>
#include <string.h>
#include <zephyr.h>

#include "bsec_datatypes.h"
#include "bsec_interface.h"

typedef struct
{
    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t len;
} bsec_outputs_t;

typedef struct
{
    bsec_version_t version;
    int64_t next_call;
    int8_t bme688_status;
    bsec_library_return_t status;
    int64_t output_timestamp;

    struct bme68x_dev _bme68x_dev;
    struct bme68x_data _bme68x_data;
    float _temp_offset;
    uint32_t _millis_overflow_counter;
    uint32_t _last_time;
    uint8_t *_bsec_config;
    bool _valid_bsec_state;
    uint8_t _bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    bsec_bme_settings_t _bsec_bme_settings;
    bsec_sensor_configuration_t _virtual_sensors[BSEC_NUMBER_OUTPUTS];
    bsec_sensor_configuration_t _sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t _n_sensor_settings;
    bsec_outputs_t _outputs;

} bsec_dev_t;

typedef struct
{
    uint8_t i2c_addr;
    const struct device *i2c_dev;

} bsec_i2c_dev_t;

/**
 * @brief Function to initialize the BSEC library and the BME688 sensor
 */
void bsec_dev_init(bsec_dev_t *bsec_dev, const struct device *i2c_dev, uint8_t i2c_addr);

/**
 * @brief Function that sets the desired sensors and the sample rates
 */
void bsec_dev_update_subscription(bsec_dev_t *bsec_dev, bsec_virtual_sensor_t sensorList[], uint8_t n_sensors, float sample_rate);

/**
 * @brief Callback from the user to trigger reading of data from the BME688, process and store outputs
 */
bool bsec_dev_run(bsec_dev_t *bsec_dev, int64_t time_ms);

/**
 * @brief Function to get the state of the algorithm to save to non-volatile memory
 */
void bsec_dev_get_state(bsec_dev_t *bsec_dev, uint8_t *state);

/**
 * @brief Function to set the state of the algorithm from non-volatile memory
 */
void bsec_dev_set_state(bsec_dev_t *bsec_dev, uint8_t *state);

/**
 * @brief Function to set the configuration of the algorithm from memory
 */
void bsec_dev_set_config(bsec_dev_t *bsec_dev, const uint8_t *config);

/**
 * @brief Function to set the temperature offset
 * @param temp_offset: Temperature offset in degree Celsius
 */
void bsec_dev_set_temperature_offset(bsec_dev_t *bsec_dev, float temp_offset);

void bsec_dev_zero_outputs(bsec_dev_t *bsec_dev);

void bsec_dev_delay_us(uint32_t period, void *intf_ptr);
int8_t bsec_dev_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
int8_t bsec_dev_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);