#include "soil_sensor.h"

#include "fdc1004.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define SOIL_SENSOR_I2C_NODE DT_NODELABEL(soil_sensor)
#define SOIL_SENSOR_ONE_SHOT_POLL_DELAY_MS 2
#define SOIL_SENSOR_ONE_SHOT_POLL_RETRIES 25

static const struct i2c_dt_spec soil_sensor_i2c = I2C_DT_SPEC_GET(SOIL_SENSOR_I2C_NODE);

static struct fdc1004_device soil_sensor_dev;
static int32_t raw_values[SOIL_SENSOR_COUNT];
static float pf_values[SOIL_SENSOR_COUNT];
static bool initialized;
static bool new_data_ready;
static const enum fdc1004_channel logical_pad_inputs[SOIL_SENSOR_COUNT] = {
	FDC1004_CHANNEL_1,
	FDC1004_CHANNEL_2,
	FDC1004_CHANNEL_3,
};

static int soil_sensor_i2c_write(const void *context,
				 uint8_t address,
				 const uint8_t *data,
				 size_t len)
{
	const struct i2c_dt_spec *i2c = context;

	if ((i2c == NULL) || !device_is_ready(i2c->bus)) {
		return -ENODEV;
	}

	return i2c_write_dt(i2c, data, len);
}

static int soil_sensor_i2c_write_read(const void *context,
				      uint8_t address,
				      const uint8_t *write_buf,
				      size_t write_len,
				      uint8_t *read_buf,
				      size_t read_len)
{
	const struct i2c_dt_spec *i2c = context;

	if ((i2c == NULL) || !device_is_ready(i2c->bus)) {
		return -ENODEV;
	}

	return i2c_write_read_dt(i2c, write_buf, write_len, read_buf, read_len);
}

static int soil_sensor_configure_device(void)
{
	uint16_t manufacturer_id;
	uint16_t device_id;
	struct fdc1004_measurement_config measurement_config;
	int ret;

	ret = fdc1004_read_manufacturer_id(&soil_sensor_dev, &manufacturer_id);
	if (ret < 0) {
		return ret;
	}

	ret = fdc1004_read_device_id(&soil_sensor_dev, &device_id);
	if (ret < 0) {
		return ret;
	}

	ret = fdc1004_reset(&soil_sensor_dev);
	if (ret < 0) {
		return ret;
	}

	for (size_t i = 0; i < SOIL_SENSOR_COUNT; ++i) {
		measurement_config.positive_input = logical_pad_inputs[i];
		measurement_config.negative_input = FDC1004_CHANNEL_4;

		ret = fdc1004_configure_measurement(&soil_sensor_dev, i, &measurement_config);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int soil_sensor_start_one_shot(uint8_t measurement_index)
{
	bool enabled[FDC1004_MEASUREMENT_COUNT] = { false, false, false, false };

	if (measurement_index >= SOIL_SENSOR_COUNT) {
		return -EINVAL;
	}

	enabled[measurement_index] = true;

	return fdc1004_start_measurements(&soil_sensor_dev,
					  FDC1004_RATE_100_SPS,
					  false,
					  enabled);
}

static int soil_sensor_wait_for_one_shot(uint8_t measurement_index)
{
	uint16_t config;
	int ret;

	for (size_t attempt = 0; attempt < SOIL_SENSOR_ONE_SHOT_POLL_RETRIES; ++attempt) {
		ret = fdc1004_read_config(&soil_sensor_dev, &config);
		if (ret < 0) {
			return ret;
		}

		if (fdc1004_is_measurement_done(config, measurement_index)) {
			return 0;
		}

		k_sleep(K_MSEC(SOIL_SENSOR_ONE_SHOT_POLL_DELAY_MS));
	}

	return -ETIMEDOUT;
}

int soil_sensor_sample(void)
{
	int ret;

	if (!initialized) {
		return -EACCES;
	}

	for (size_t i = 0; i < SOIL_SENSOR_COUNT; ++i) {
		ret = soil_sensor_start_one_shot(i);
		if (ret < 0) {
			new_data_ready = false;
			return ret;
		}

		ret = soil_sensor_wait_for_one_shot(i);
		if (ret < 0) {
			new_data_ready = false;
			return ret;
		}

		ret = fdc1004_read_raw_measurement(&soil_sensor_dev, i, &raw_values[i]);
		if (ret < 0) {
			new_data_ready = false;
			return ret;
		}

		pf_values[i] = fdc1004_raw_to_pf(raw_values[i]);
	}

	new_data_ready = true;

	return 0;
}

int32_t soil_sensor_get_raw(uint8_t channel)
{
	if (channel >= SOIL_SENSOR_COUNT) {
		return 0;
	}

	return raw_values[channel];
}

float soil_sensor_get_pf(uint8_t channel)
{
	if (channel >= SOIL_SENSOR_COUNT) {
		return 0.0f;
	}

	return pf_values[channel];
}

bool soil_sensor_has_new_data(void)
{
	return new_data_ready;
}

int soil_sensor_init(void)
{
	struct fdc1004_io io = {
		.write = soil_sensor_i2c_write,
		.write_read = soil_sensor_i2c_write_read,
		.context = (const void *)&soil_sensor_i2c,
	};
	int ret;

	if (!device_is_ready(soil_sensor_i2c.bus)) {
		return -ENODEV;
	}

	ret = fdc1004_init(&soil_sensor_dev, &io);
	if (ret < 0) {
		return ret;
	}

	ret = soil_sensor_configure_device();
	if (ret < 0) {
		return ret;
	}

	(void)memset(raw_values, 0, sizeof(raw_values));
	(void)memset(pf_values, 0, sizeof(pf_values));
	initialized = true;
	new_data_ready = false;

	return 0;
}
