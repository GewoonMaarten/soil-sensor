#ifndef FDC1004_H
#define FDC1004_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FDC1004_I2C_ADDRESS 0x50U
#define FDC1004_CHANNEL_COUNT 4U
#define FDC1004_MEASUREMENT_COUNT 4U

enum fdc1004_channel {
	FDC1004_CHANNEL_1 = 0,
	FDC1004_CHANNEL_2,
	FDC1004_CHANNEL_3,
	FDC1004_CHANNEL_4,
};

enum fdc1004_rate {
	FDC1004_RATE_100_SPS = 1,
	FDC1004_RATE_200_SPS = 2,
	FDC1004_RATE_400_SPS = 3,
};

typedef int (*fdc1004_i2c_write_fn)(const void *context,
				    uint8_t address,
				    const uint8_t *data,
				    size_t len);
typedef int (*fdc1004_i2c_write_read_fn)(const void *context,
					 uint8_t address,
					 const uint8_t *write_buf,
					 size_t write_len,
					 uint8_t *read_buf,
					 size_t read_len);

struct fdc1004_io {
	fdc1004_i2c_write_fn write;
	fdc1004_i2c_write_read_fn write_read;
	const void *context;
};

struct fdc1004_device {
	struct fdc1004_io io;
	uint8_t address;
};

struct fdc1004_measurement_config {
	enum fdc1004_channel positive_input;
	enum fdc1004_channel negative_input;
};

int fdc1004_init(struct fdc1004_device *dev, const struct fdc1004_io *io);
int fdc1004_reset(struct fdc1004_device *dev);
int fdc1004_read_manufacturer_id(struct fdc1004_device *dev, uint16_t *manufacturer_id);
int fdc1004_read_device_id(struct fdc1004_device *dev, uint16_t *device_id);
int fdc1004_configure_measurement(struct fdc1004_device *dev,
				  uint8_t measurement_index,
				  const struct fdc1004_measurement_config *config);
int fdc1004_start_measurements(struct fdc1004_device *dev,
			       enum fdc1004_rate rate,
			       bool continuous,
			       const bool enabled[FDC1004_MEASUREMENT_COUNT]);
int fdc1004_read_config(struct fdc1004_device *dev, uint16_t *config);
bool fdc1004_is_measurement_done(uint16_t config, uint8_t measurement_index);
int fdc1004_read_raw_measurement(struct fdc1004_device *dev,
				 uint8_t measurement_index,
				 int32_t *raw_measurement);
float fdc1004_raw_to_pf(int32_t raw_measurement);

#endif
