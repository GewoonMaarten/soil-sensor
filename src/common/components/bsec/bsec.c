#include "bsec.h"

void bsec_dev_init(bsec_dev_t *bsec_dev, const struct device *i2c_dev, uint8_t i2c_addr)
{
    bsec_i2c_dev_t bsec_i2c_dev = {
        .i2c_addr = i2c_addr,
        .i2c_dev = i2c_dev,
    };
    bsec_virtual_sensor_t sensor_ids[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_COMPENSATED_GAS,
        BSEC_OUTPUT_GAS_PERCENTAGE};
    const uint8_t sensor_ids_size = sizeof sensor_ids / sizeof sensor_ids[0];

    bsec_dev->bme688_status = BME68X_OK;
    bsec_dev->next_call = 0;
    bsec_dev->output_timestamp = 0;
    bsec_dev->status = BSEC_OK;
    bsec_dev->version.major = 0;
    bsec_dev->version.major_bugfix = 0;
    bsec_dev->version.minor = 0;
    bsec_dev->version.minor_bugfix = 0;

    bsec_dev->_bsec_config = NULL;
    bsec_dev->_last_time = 0;
    bsec_dev->_millis_overflow_counter = 0;
    bsec_dev->_n_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    bsec_dev->_temp_offset = 0;

    bsec_dev_zero_outputs(bsec_dev);

    bsec_dev->_bme68x_dev.intf = BME68X_I2C_INTF;
    bsec_dev->_bme68x_dev.read = bsec_dev_i2c_read;
    bsec_dev->_bme68x_dev.write = bsec_dev_i2c_write;
    bsec_dev->_bme68x_dev.delay_us = bsec_dev_delay_us;
    bsec_dev->_bme68x_dev.amb_temp = 25;
    bsec_dev->_bme68x_dev.intf_ptr = &bsec_i2c_dev;

    for (uint8_t i = 0; i < sensor_ids_size; i++)
    {
        bsec_dev->_virtual_sensors->sensor_id = sensor_ids[i];
        bsec_dev->_virtual_sensors->sample_rate = BSEC_SAMPLE_RATE_DISABLED;
    }

    bsec_dev->status = bsec_init();
    bsec_dev->bme688_status = bme68x_init(&bsec_dev->_bme68x_dev);

    bsec_get_version(&bsec_dev->version);
}

void bsec_dev_update_subscription(
    bsec_dev_t *bsec_dev, bsec_virtual_sensor_t sensor_list[], uint8_t n_sensors, float sample_rate)
{
    for (uint8_t i = 0; i < n_sensors; i++)
    {
        for (uint8_t j = 0; j < BSEC_NUMBER_OUTPUTS; j++)
        {
            if (bsec_dev->_virtual_sensors[j].sensor_id == sensor_list[i])
            {
                bsec_dev->_virtual_sensors[j].sample_rate = sample_rate;
            }
        }
    }

    bsec_dev->status = bsec_update_subscription(
        bsec_dev->_virtual_sensors,
        BSEC_NUMBER_OUTPUTS,
        bsec_dev->_sensor_settings,
        &bsec_dev->_n_sensor_settings);

    return;
}

bool bsec_dev_run(bsec_dev_t *bsec_dev, int64_t time_ms)
{
    bool newData = false;
    /* Check if the time has arrived to call do_steps() */
    int64_t call_time_ms = time_ms;

    if (call_time_ms < 0) /* Use millis */
    {
        call_time_ms = getTimeMs();
    }

    if (call_time_ms >= bsec_dev->next_call)
    {
        if (bsec_dev->_valid_bsec_state)
        {
            bsec_dev_set_state(bsec_dev, bsec_dev->_bsec_state);
        }

        bsec_dev->_n_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
        bsec_dev->status = bsec_update_subscription(
            bsec_dev->_virtual_sensors,
            BSEC_NUMBER_OUTPUTS,
            bsec_dev->_sensor_settings,
            &bsec_dev->_n_sensor_settings);

        bsec_bme_settings_t bme68x_settings;

        int64_t call_time_ns = call_time_ms * INT64_C(1000000);

        bsec_dev->status = bsec_sensor_control(call_time_ns, &bme68x_settings);
        if (bsec_dev->status < BSEC_OK)
        {
            return false;
        }

        bsec_dev->next_call = bme68x_settings.next_call / INT64_C(1000000); /* Convert from ns to ms */

        bsec_dev->bme688_status = setBme680Config(bme68x_settings);
        if (bsec_dev->bme688_status != BME68X_OK)
        {
            return false;
        }

        bsec_dev->bme688_status = bme680_set_sensor_mode(&bsec_dev->_bme68x_dev);
        if (bsec_dev->bme688_status != BME68X_OK)
        {
            return false;
        }

        /* Wait for measurement to complete */
        uint16_t meas_dur = 0;

        bme68x_get_meas_dur()

            // bme680_get_profile_dur(&meas_dur, &bsec_dev->_bme68x_dev);
            bsec_dev->_bme68x_dev.delay_us(meas_dur, NULL);

        newData = readProcessData(call_time_ns + (meas_dur * INT64_C(1000000)), bme68x_settings);

        uint8_t workBuffer[BSEC_MAX_STATE_BLOB_SIZE];
        uint32_t n_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;
        bsec_dev->status = bsec_get_state(0,
                                          bsec_dev->_bsec_state,
                                          BSEC_MAX_STATE_BLOB_SIZE,
                                          workBuffer,
                                          BSEC_MAX_STATE_BLOB_SIZE,
                                          &n_serialized_state);
        bsec_dev->_valid_bsec_state = true;
    }

    return newData;
}

void bsec_dev_zero_outputs(bsec_dev_t *bsec_dev)
{
    memset(&bsec_dev->_outputs, 0, sizeof(bsec_dev->_outputs));
    bsec_dev->_valid_bsec_state = false;
}

void bsec_dev_delay_us(uint32_t period, void *intf_ptr)
{
    k_usleep(period);
}

int8_t bsec_dev_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bsec_i2c_dev_t bsec_dev = *(bsec_i2c_dev_t *)intf_ptr;

    unsigned char reg[len + 1];
    reg[0] = reg_addr;
    memcpy(reg + 1, reg_data, len);

    return i2c_write(bsec_dev.i2c_dev, reg, len + 1, bsec_dev.i2c_addr);
}

int8_t bsec_dev_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bsec_i2c_dev_t bsec_dev = *(bsec_i2c_dev_t *)intf_ptr;

    unsigned char buf[1] = {reg_addr};
    i2c_write(bsec_dev.i2c_dev, buf, 1, bsec_dev.i2c_addr);
    return i2c_read(bsec_dev.i2c_dev, reg_data, len, bsec_dev.i2c_addr);
}