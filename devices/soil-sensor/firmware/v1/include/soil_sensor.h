#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

#define SOIL_SENSOR_COUNT 3

int soil_sensor_init(void);
int soil_sensor_sample(void);

int32_t soil_sensor_get_raw(uint8_t channel);
float soil_sensor_get_pf(uint8_t channel);
bool soil_sensor_has_new_data(void);

#endif