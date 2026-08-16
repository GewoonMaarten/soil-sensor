#include "fdc1004.h"
#include <errno.h>

#define FDC1004_REG_MEAS1_MSB 0x00U
#define FDC1004_REG_MEAS1_LSB 0x01U
#define FDC1004_REG_MEAS2_MSB 0x02U
#define FDC1004_REG_MEAS2_LSB 0x03U
#define FDC1004_REG_MEAS3_MSB 0x04U
#define FDC1004_REG_MEAS3_LSB 0x05U
#define FDC1004_REG_MEAS4_MSB 0x06U
#define FDC1004_REG_MEAS4_LSB 0x07U

#define FDC1004_REG_CONF_MEAS1 0x08U
#define FDC1004_REG_CONF_MEAS2 0x09U
#define FDC1004_REG_CONF_MEAS3 0x0AU
#define FDC1004_REG_CONF_MEAS4 0x0BU
#define FDC1004_REG_FDC_CONF 0x0CU
#define FDC1004_REG_MANUFACTURER_ID 0xFEU
#define FDC1004_REG_DEVICE_ID 0xFFU

#define FDC1004_DEVICE_ID 0x1004U
#define FDC1004_MANUFACTURER_ID 0x5449U
#define FDC1004_FDC_CONF_RST_BIT 15U
#define FDC1004_RESET_POLL_RETRIES 32U

static const uint8_t measurement_registers[FDC1004_MEASUREMENT_COUNT][3] = {
	{ FDC1004_REG_MEAS1_MSB, FDC1004_REG_MEAS1_LSB, FDC1004_REG_CONF_MEAS1 },
	{ FDC1004_REG_MEAS2_MSB, FDC1004_REG_MEAS2_LSB, FDC1004_REG_CONF_MEAS2 },
	{ FDC1004_REG_MEAS3_MSB, FDC1004_REG_MEAS3_LSB, FDC1004_REG_CONF_MEAS3 },
	{ FDC1004_REG_MEAS4_MSB, FDC1004_REG_MEAS4_LSB, FDC1004_REG_CONF_MEAS4 },
};

static uint8_t fdc1004_measurement_enable_bit(uint8_t measurement_index)
{
	return 7U - measurement_index;
}

static uint8_t fdc1004_measurement_done_bit(uint8_t measurement_index)
{
	return 3U - measurement_index;
}

static int fdc1004_write_register(struct fdc1004_device *dev,
				  uint8_t reg_addr,
				  const uint8_t *data,
				  size_t len)
{
	uint8_t buf[3];

	if ((dev == NULL) || (dev->io.write == NULL) || (data == NULL) || (len > 2U)) {
		return -EINVAL;
	}

	buf[0] = reg_addr;
	for (size_t i = 0; i < len; ++i) {
		buf[i + 1U] = data[i];
	}

	return dev->io.write(dev->io.context, dev->address, buf, len + 1U);
}

static int fdc1004_read_register(struct fdc1004_device *dev,
				 uint8_t reg_addr,
				 uint8_t *data,
				 size_t len)
{
	if ((dev == NULL) || (dev->io.write_read == NULL) || (data == NULL) || (len == 0U)) {
		return -EINVAL;
	}

	return dev->io.write_read(dev->io.context, dev->address, &reg_addr, 1U, data, len);
}

static int fdc1004_read_u16(struct fdc1004_device *dev, uint8_t reg_addr, uint16_t *value)
{
	uint8_t buf[2];
	int ret;

	if (value == NULL) {
		return -EINVAL;
	}

	ret = fdc1004_read_register(dev, reg_addr, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	*value = ((uint16_t)buf[0] << 8) | buf[1];

	return 0;
}

int fdc1004_init(struct fdc1004_device *dev, const struct fdc1004_io *io)
{
	if ((dev == NULL) || (io == NULL) || (io->write == NULL) || (io->write_read == NULL)) {
		return -EINVAL;
	}

	dev->io = *io;
	dev->address = FDC1004_I2C_ADDRESS;

	return 0;
}

int fdc1004_reset(struct fdc1004_device *dev)
{
	uint16_t config;
	uint8_t buf[2] = {
		(uint8_t)(1U << (FDC1004_FDC_CONF_RST_BIT - 8U)),
		0U,
	};
	int ret;

	ret = fdc1004_write_register(dev, FDC1004_REG_FDC_CONF, buf, sizeof(buf));
	if (ret < 0) {
		return ret;
	}

	for (size_t i = 0; i < FDC1004_RESET_POLL_RETRIES; ++i) {
		ret = fdc1004_read_u16(dev, FDC1004_REG_FDC_CONF, &config);
		if (ret < 0) {
			return ret;
		}

		if ((config & (1U << FDC1004_FDC_CONF_RST_BIT)) == 0U) {
			return 0;
		}
	}

	return -EIO;
}

int fdc1004_read_manufacturer_id(struct fdc1004_device *dev, uint16_t *manufacturer_id)
{
	int ret = fdc1004_read_u16(dev, FDC1004_REG_MANUFACTURER_ID, manufacturer_id);

	if ((ret == 0) && (*manufacturer_id != FDC1004_MANUFACTURER_ID)) {
		return -ENODEV;
	}

	return ret;
}

int fdc1004_read_device_id(struct fdc1004_device *dev, uint16_t *device_id)
{
	int ret = fdc1004_read_u16(dev, FDC1004_REG_DEVICE_ID, device_id);

	if ((ret == 0) && (*device_id != FDC1004_DEVICE_ID)) {
		return -ENODEV;
	}

	return ret;
}

int fdc1004_configure_measurement(struct fdc1004_device *dev,
				  uint8_t measurement_index,
				  const struct fdc1004_measurement_config *config)
{
	uint16_t register_value;
	uint8_t buf[2];

	if ((measurement_index >= FDC1004_MEASUREMENT_COUNT) || (config == NULL) ||
	    (config->positive_input >= FDC1004_CHANNEL_COUNT) ||
	    (config->negative_input >= FDC1004_CHANNEL_COUNT)) {
		return -EINVAL;
	}

	register_value = ((uint16_t)(config->positive_input) << 13) |
			 ((uint16_t)(config->negative_input) << 10);
	buf[0] = (uint8_t)(register_value >> 8);
	buf[1] = (uint8_t)register_value;

	return fdc1004_write_register(dev,
				      measurement_registers[measurement_index][2],
				      buf,
				      sizeof(buf));
}

int fdc1004_start_measurements(struct fdc1004_device *dev,
			       enum fdc1004_rate rate,
			       bool continuous,
			       const bool enabled[FDC1004_MEASUREMENT_COUNT])
{
	uint16_t register_value;
	uint8_t buf[2];

	if ((enabled == NULL) || (rate < FDC1004_RATE_100_SPS) || (rate > FDC1004_RATE_400_SPS)) {
		return -EINVAL;
	}

	register_value = (uint16_t)rate << 10;
	register_value |= (uint16_t)continuous << 8;

	for (size_t i = 0; i < FDC1004_MEASUREMENT_COUNT; ++i) {
		register_value |= (uint16_t)enabled[i] << fdc1004_measurement_enable_bit(i);
	}

	buf[0] = (uint8_t)(register_value >> 8);
	buf[1] = (uint8_t)register_value;

	return fdc1004_write_register(dev, FDC1004_REG_FDC_CONF, buf, sizeof(buf));
}

int fdc1004_read_config(struct fdc1004_device *dev, uint16_t *config)
{
	return fdc1004_read_u16(dev, FDC1004_REG_FDC_CONF, config);
}

bool fdc1004_is_measurement_done(uint16_t config, uint8_t measurement_index)
{
	if (measurement_index >= FDC1004_MEASUREMENT_COUNT) {
		return false;
	}

	return ((config >> fdc1004_measurement_done_bit(measurement_index)) & 0x1U) != 0U;
}

int fdc1004_read_raw_measurement(struct fdc1004_device *dev,
				 uint8_t measurement_index,
				 int32_t *raw_measurement)
{
	uint16_t measurement_msb;
	uint16_t measurement_lsb;
	int32_t value;
	int ret;

	if ((measurement_index >= FDC1004_MEASUREMENT_COUNT) || (raw_measurement == NULL)) {
		return -EINVAL;
	}

	ret = fdc1004_read_u16(dev, measurement_registers[measurement_index][0], &measurement_msb);
	if (ret < 0) {
		return ret;
	}

	ret = fdc1004_read_u16(dev, measurement_registers[measurement_index][1], &measurement_lsb);
	if (ret < 0) {
		return ret;
	}

	value = ((int32_t)measurement_msb << 8) | ((int32_t)measurement_lsb >> 8);
	if ((value & (1L << 23)) != 0) {
		value |= ~0x00FFFFFF;
	}

	*raw_measurement = value;

	return 0;
}

float fdc1004_raw_to_pf(int32_t raw_measurement)
{
	return (float)raw_measurement / 524288.0f;
}
