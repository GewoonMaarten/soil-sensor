#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble.h"
#include "sensor.h"

static const char *TAG = __FILE__;

static int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int gatt_svr_chr_access_sensor(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /*** Service: Device Info. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                /*** Characteristic: Device ID. */
                .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_VERSION_UUID),
                .access_cb = gatt_svr_chr_access_device_info,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                0, /* No more characteristics in this service. */
            },
        },
    },
    {
        /*** Service: Sensor. */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(GATT_SENSOR_UUID),
        .characteristics = (struct ble_gatt_chr_def[]){
            // {
            //     /*** Characteristic: Temperature. */
            //     .uuid = BLE_UUID16_DECLARE(GATT_SENSOR_TEMP_UUID),
            //     .access_cb = gatt_svr_chr_access_sensor,
            //     .flags = BLE_GATT_CHR_F_READ,
            // },
            // {
            //     /*** Characteristic: Humidity. */
            //     .uuid = BLE_UUID16_DECLARE(GATT_SENSOR_HUMD_UUID),
            //     .access_cb = gatt_svr_chr_access_sensor,
            //     .flags = BLE_GATT_CHR_F_READ,
            // },
            {
                /*** Characteristic: Battery voltage. */
                .uuid = BLE_UUID16_DECLARE(GATT_SENSOR_VOLT_UUID),
                .access_cb = gatt_svr_chr_access_sensor,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /*** Characteristic: 3 Moisture levels. */
                .uuid = BLE_UUID16_DECLARE(GATT_SENSOR_MOIS_UUID),
                .access_cb = gatt_svr_chr_access_sensor,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /*** Characteristic: 3 Moisture levels (calibrated). */
                .uuid = BLE_UUID16_DECLARE(GATT_SENSOR_MOIS_CAL_UUID),
                .access_cb = gatt_svr_chr_access_sensor,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                0, /* No more characteristics in this service. */
            },
        },
    },
    {
        0, /* No more services. */
    },
};

static int
gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    if (uuid == GATT_DEVICE_VERSION_UUID)
    {
        uint8_t dev_id = 0x32;
        rc = os_mbuf_append(ctxt->om, &dev_id, sizeof(uint8_t));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

static int
gatt_svr_chr_access_sensor(uint16_t conn_handle, uint16_t attr_handle,
                           struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint16_t uuid;
    int rc;

    uuid = ble_uuid_u16(ctxt->chr->uuid);

    switch (uuid)
    {
    // case GATT_SENSOR_TEMP_UUID:
    // {
    //     hdc1080_median_temperature(hdc1080_device, 3);

    //     ble_packed_t package = _ble_pack_float(hdc1080_device->temperature);

    //     // set characteristic value
    //     rc = os_mbuf_append(ctxt->om, &package, sizeof(package));
    //     return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    // }
    // case GATT_SENSOR_HUMD_UUID:
    // {
    //     hdc1080_median_humidity(hdc1080_device, 3);

    //     ble_packed_t package = _ble_pack_float(hdc1080_device->humidity);

    //     // set characteristic value
    //     rc = os_mbuf_append(ctxt->om, &package, sizeof(package));
    //     return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    // }
    case GATT_SENSOR_VOLT_UUID:
    {
        ESP_LOGD(TAG, "voltage raw: %dmV, voltage: %dmV", get_voltage(), (uint16_t)(get_voltage() / ADC_VOLT_DIV));
        ble_packed_t package = _ble_pack_uint16(get_voltage() / ADC_VOLT_DIV);

        // set characteristic value
        rc = os_mbuf_append(ctxt->om, &package, sizeof(package));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    case GATT_SENSOR_MOIS_UUID:
    {
        moisture_read_sensor(moisture_sensor);
        ble_packed_t package[3] = {
            _ble_pack_uint16(moisture_sensor->sensor_1_raw),
            _ble_pack_uint16(moisture_sensor->sensor_2_raw),
            _ble_pack_uint16(moisture_sensor->sensor_3_raw),
        };
        // set characteristic value
        rc = os_mbuf_append(ctxt->om, &package, sizeof(package));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    case GATT_SENSOR_MOIS_CAL_UUID:
    {
        moisture_read_sensor(moisture_sensor);
        ble_packed_t package[3] = {
            _ble_pack_float(moisture_sensor->sensor_1_cal),
            _ble_pack_float(moisture_sensor->sensor_2_cal),
            _ble_pack_float(moisture_sensor->sensor_3_cal),
        };
        // set characteristic value
        rc = os_mbuf_append(ctxt->om, &package, sizeof(package));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    default:
        break;
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0)
    {
        return rc;
    }

    return 0;
}