#pragma once

#define BLE_ADVERTISEMENT_TIME 1 * 60 * 1000 // in milliseconds

#define GATT_DEVICE_INFO_UUID 0x180A
#define GATT_DEVICE_VERSION_UUID 0x3e2f

#define GATT_SENSOR_UUID 0x181A
#define GATT_SENSOR_TEMP_UUID 0x2A6E
#define GATT_SENSOR_HUMD_UUID 0x2A6F
#define GATT_SENSOR_VOLT_UUID 0x2B18
#define GATT_SENSOR_MOIS_UUID 0x290C
#define GATT_SENSOR_MOIS_CAL_UUID 0x0002

typedef struct
{
    uint8_t value[2];
} ble_packed_t;

struct ble_gatt_register_ctxt;

int gatt_svr_init(void);

void ble_host_task(void *param);
void ble_on_sync(void);
void ble_on_reset(int reason);

ble_packed_t _ble_pack_float(float value);
ble_packed_t _ble_pack_uint16(uint16_t value);
