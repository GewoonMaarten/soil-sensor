#pragma once

#include <bme68x.h>
#include <bme68x_defs.h>
#include <string.h>

#include "bsec_datatypes.h"
#include "bsec_interface.h"

int8_t bme68x_interface_init(struct bme68x_dev *bme);
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bme68x_delay_us(uint32_t period, void *intf_ptr);
void bme68x_check_rslt(const char api_name[], int8_t rslt);

// typedef struct
// {
//     bsec_output_t output[BSEC_NUMBER_OUTPUTS];
//     uint8_t nOutputs;
// } bsec_outputs_t;

// typedef struct bsec_dev_t;

// typedef void (*bsec_callback)(struct bme68x_data data, const bsec_outputs_t outputs, const bsec_dev_t bsec);

// typedef struct
// {
//     struct bme68x_dev sensor;
//     /* Stores the version of the BSEC algorithm */
//     bsec_version_t version;
//     bsec_library_return_t status;

//     bsec_bme_settings_t bmeConf;

//     bsec_callback newDataCallback;

//     bsec_outputs_t outputs;
//     /* operating mode of sensor */
//     uint8_t opMode;

//     float extTempOffset;
//     /** Global variables to help create a millisecond timestamp that doesn't overflow every 51 days.
//      * If it overflows, it will have a negative value. Something that should never happen.
//      */
//     uint32_t ovfCounter;

//     uint32_t lastMillis;

// } bsec_dev_t;

// bool begin(uint8_t i2c_addr, const struct device *i2c_dev, bme68x_delay_us_fptr_t idleTask);

// typedef struct
// {
//     uint8_t i2c_addr;
//     const struct device *i2c_dev;

// } bsec_i2c_dev_t;

// /**
//  * @brief Function to initialize the BSEC library and the BME688 sensor
//  */
// void bsec_dev_init(bsec_dev_t *bsec_dev, const struct device *i2c_dev, uint8_t i2c_addr);

// /**
//  * @brief Function that sets the desired sensors and the sample rates
//  */
// void bsec_dev_update_subscription(bsec_dev_t *bsec_dev, bsec_virtual_sensor_t sensorList[], uint8_t n_sensors, float sample_rate);

// /**
//  * @brief Callback from the user to trigger reading of data from the BME688, process and store outputs
//  */
// bool bsec_dev_run(bsec_dev_t *bsec_dev, int64_t time_ms);

// /**
//  * @brief Function to get the state of the algorithm to save to non-volatile memory
//  */
// void bsec_dev_get_state(bsec_dev_t *bsec_dev, uint8_t *state);

// /**
//  * @brief Function to set the state of the algorithm from non-volatile memory
//  */
// void bsec_dev_set_state(bsec_dev_t *bsec_dev, uint8_t *state);

// /**
//  * @brief Function to set the configuration of the algorithm from memory
//  */
// void bsec_dev_set_config(bsec_dev_t *bsec_dev, const uint8_t *config);

// /**
//  * @brief Function to set the temperature offset
//  * @param temp_offset: Temperature offset in degree Celsius
//  */
// void bsec_dev_set_temperature_offset(bsec_dev_t *bsec_dev, float temp_offset);

// void bsec_dev_zero_outputs(bsec_dev_t *bsec_dev);

// void bsec_dev_delay_us(uint32_t period, void *intf_ptr);
// int8_t bsec_dev_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
// int8_t bsec_dev_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);