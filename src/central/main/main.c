/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include "esp_log.h"
#include "esp_event.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_mac.h"

/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"

#include "blecent.h"
#include "mqtt_client.h"
#include "cJSON.h"

#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include "bme68x.h"

/* WiFi */
#include "wifi.h"

#include "esp_tls.h"
#include "esp_http_client.h"

#include "driver/i2c.h"

#include "bsec.h"

static const uint8_t num_addrs = 3;
/* lower case only! */
static const char addrs[3][17 + 1] = {
    {"98:cd:ac:eb:16:be"},
    {"98:cd:ac:eb:16:22"},
    {"98:cd:ac:eb:16:c2"},
};

static const char *tag = "BLE_CENTRAL";

static int blecent_gap_event(struct ble_gap_event *event, void *arg);
void ble_store_config_init(void);
static esp_mqtt_client_handle_t mqtt_client;

static void
http_post_data(char *addr, int voltage, uint16_t m1, uint16_t m2, uint16_t m3)
{
    esp_http_client_config_t config = {
        .url = "http://storage-1.lan:8086/write?db=telegraf&precision=s&u=telegraf&p=metricsmetricsmetrics",
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    char data[128];
    snprintf(data, 128, "sensors,device=soil_sensor,addr=%s voltage=%di,moisture1=%d,moisture2=%d,moisture3=%d", addr, voltage, m1, m2, m3);

    esp_http_client_set_post_field(client, data, strlen(data));
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(tag, "HTTP POST Status = %d, content_length = %d",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(tag, "HTTP POST request failed: %s", esp_err_to_name(err));
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

/**
 * Application callback.  Called when the read of the ANS Supported New Alert
 * Category characteristic has completed.
 */
static int
blecent_on_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    ESP_LOGD(tag, "Read complete; status=%d conn_handle=%d", error->status,
             conn_handle);

    if (error->status != 0)
    {
        return -1;
    }

    struct ble_gap_conn_desc *desc = (struct ble_gap_conn_desc *)malloc(sizeof(struct ble_gap_conn_desc));
    ble_gap_conn_find(conn_handle, desc);

    uint16_t mv = attr->om->om_data[1] << 8 | attr->om->om_data[0];
    uint16_t m1 = attr->om->om_data[3] << 8 | attr->om->om_data[2];
    uint16_t m2 = attr->om->om_data[5] << 8 | attr->om->om_data[4];
    uint16_t m3 = attr->om->om_data[7] << 8 | attr->om->om_data[6];
    ESP_LOGD(tag, "voltage: %dmV, m1: %d, m2: %d, m3: %d", mv, m1, m2, m3);

    // http_post_data(addr_str(desc->peer_ota_addr.val), mv, m1, m2, m3);
    char *device_addr = addr_str(desc->peer_ota_addr.val);

    char sensor_names[3][19 + 1] = {
        {"soilSensorLevel1"},
        {"soilSensorLevel2"},
        {"soilSensorLevel3"},
    };
    char sensor_value_template[35 + 1];
    char sensor_unique_id[37 + 1];
    char state_topic[44 + 1];
    char config_topic[62 + 1];
    char device_name[23 + 1];

    for (uint8_t i = 0; i < 3; i++)
    {
        snprintf(sensor_value_template, 35 + 1, "{{ value_json.%s}}", sensor_names[i]);
        snprintf(sensor_unique_id, 37 + 1, "%s_%s", device_addr, sensor_names[i]);
        snprintf(state_topic, 45, "homeassistant/sensor/%s/state", device_addr);
        snprintf(config_topic, 62 + 7 + 1, "homeassistant/sensor/%s/config", sensor_unique_id);
        snprintf(device_name, 23 + 1, "Plant %s", device_addr);

        // Create device JSON
        cJSON *device = cJSON_CreateObject();
        cJSON *identifiers = cJSON_AddArrayToObject(device, "identifiers");
        cJSON_AddItemToArray(identifiers, cJSON_CreateString(device_addr));
        cJSON_AddStringToObject(device, "manufacturer", "Maarten de Klerk");
        cJSON_AddStringToObject(device, "model", "BLE Soil Sensor");
        cJSON_AddStringToObject(device, "name", device_name);

        // Create payload JSON
        cJSON *payload = cJSON_CreateObject();
        cJSON_AddStringToObject(payload, "state_class", "measurement");
        cJSON_AddStringToObject(payload, "state_topic", state_topic);
        cJSON_AddItemToObject(payload, "device", device);
        cJSON_AddNumberToObject(payload, "qos", 1);
        cJSON_AddBoolToObject(payload, "force_update", true);
        cJSON_AddNumberToObject(payload, "expire_after", 1000);
        cJSON_AddStringToObject(payload, "value_template", sensor_value_template);
        cJSON_AddStringToObject(payload, "unique_id", sensor_unique_id);
        cJSON_AddStringToObject(payload, "name", sensor_names[i]);
        cJSON_AddStringToObject(payload, "icon", "mdi:water-percent");

        char *payload_string = cJSON_PrintUnformatted(payload);
        printf(payload_string);

        esp_mqtt_client_publish(mqtt_client, config_topic, payload_string, 0, 1, 0);

        cJSON_Delete(payload);
    }

    cJSON *state_payload = cJSON_CreateObject();
    cJSON_AddNumberToObject(state_payload, sensor_names[0], m1);
    cJSON_AddNumberToObject(state_payload, sensor_names[1], m2);
    cJSON_AddNumberToObject(state_payload, sensor_names[2], m3);

    char *payload_string = cJSON_PrintUnformatted(state_payload);
    printf(payload_string);

    esp_mqtt_client_publish(mqtt_client, state_topic, payload_string, 0, 1, 0);

    free(desc);

    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);

    return 0;
}

/**
 * Performs three GATT operations against the specified peer:
 * 1. Reads the ANS Supported New Alert Category characteristic.
 * 2. After read is completed, writes the ANS Alert Notification Control Point characteristic.
 * 3. After write is completed, subscribes to notifications for the ANS Unread Alert Status
 *    characteristic.
 *
 * If the peer does not support a required service, characteristic, or
 * descriptor, then the peer lied when it claimed support for the alert
 * notification service!  When this happens, or if a GATT procedure fails,
 * this function immediately terminates the connection.
 */
static void
blecent_read(const struct peer *peer)
{
    const struct peer_chr *chr_voltage;
    const struct peer_chr *chr_moisture;
    int rc;
    uint16_t handles[2];

    /* Read the supported-new-alert-category characteristic. */
    chr_voltage = peer_chr_find_uuid(peer,
                                     BLE_UUID16_DECLARE(0x181A),
                                     BLE_UUID16_DECLARE(0x2B18));

    if (chr_voltage == NULL)
    {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the voltage characteristic\n");
        goto err;
    }

    chr_moisture = peer_chr_find_uuid(peer,
                                      BLE_UUID16_DECLARE(0x181A),
                                      BLE_UUID16_DECLARE(0x290C));

    if (chr_voltage == NULL)
    {
        MODLOG_DFLT(ERROR, "Error: Peer doesn't support the moisture characteristic\n");
        goto err;
    }

    handles[0] = chr_voltage->chr.val_handle;
    handles[1] = chr_moisture->chr.val_handle;

    rc = ble_gattc_read_mult(peer->conn_handle, handles, 2, blecent_on_read, NULL);

    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "Error: Failed to read characteristic; rc=%d\n",
                    rc);
        goto err;
    }

    return;
err:
    /* Terminate the connection. */
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

/**
 * Called when service discovery of the specified peer has completed.
 */
static void
blecent_on_disc_complete(const struct peer *peer, int status, void *arg)
{

    if (status != 0)
    {
        /* Service discovery failed.  Terminate the connection. */
        MODLOG_DFLT(ERROR, "Error: Service discovery failed; status=%d "
                           "conn_handle=%d\n",
                    status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    /* Service discovery has completed successfully.  Now we have a complete
     * list of services, characteristics, and descriptors that the peer
     * supports.
     */
    MODLOG_DFLT(ERROR, "Service discovery complete; status=%d "
                       "conn_handle=%d\n",
                status, peer->conn_handle);

    /* Now perform three GATT procedures against the peer: read,
     * write, and subscribe to notifications.
     */
    blecent_read(peer);
}

/**
 * Initiates the GAP general discovery procedure.
 */
static void
blecent_scan(void)
{
    uint8_t own_addr_type;
    struct ble_gap_disc_params disc_params;
    int rc;

    /* Figure out address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Tell the controller to filter duplicates; we don't want to process
     * repeated advertisements from the same device.
     */
    disc_params.filter_duplicates = 1;

    /**
     * Perform a passive scan.  I.e., don't send follow-up scan requests to
     * each advertiser.
     */
    disc_params.passive = 1;

    /* Use defaults for the rest of the parameters. */
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}

/**
 * Indicates whether we should try to connect to the sender of the specified
 * advertisement.  The function returns a positive result if the device
 * advertises connectability and support for the Alert Notification service.
 */
static int
blecent_should_connect(const struct ble_gap_disc_desc *disc)
{
    uint8_t peer_addr[6];
    uint8_t i;
    struct ble_hs_adv_fields fields;
    int rc;

    /* The device has to be advertising connectability. */
    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND)
    {

        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0)
    {
        return rc;
    }

    for (i = 0; i < num_addrs; i++)
    {
        sscanf(addrs[i], "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &peer_addr[5], &peer_addr[4], &peer_addr[3],
               &peer_addr[2], &peer_addr[1], &peer_addr[0]);
        if (memcmp(peer_addr, disc->addr.val, sizeof(disc->addr.val)) == 0)
        {
            return 1;
        }
    }

    return 0;
}

/**
 * Connects to the sender of the specified advertisement of it looks
 * interesting.  A device is "interesting" if it advertises connectability and
 * support for the Alert Notification service.
 */
static void
blecent_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    uint8_t own_addr_type;
    int rc;

    /* Don't do anything if we don't care about this advertiser. */
    if (!blecent_should_connect(disc))
    {
        return;
    }

    /* Scanning must be stopped before a connection can be initiated. */
    rc = ble_gap_disc_cancel();
    if (rc != 0)
    {
        MODLOG_DFLT(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }

    /* Figure out address to use for connect (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "error determining address type; rc=%d\n", rc);
        return;
    }

    /* Try to connect the the advertiser.  Allow 30 seconds (30000 ms) for
     * timeout.
     */
    rc = ble_gap_connect(own_addr_type, &disc->addr, 30000, NULL,
                         blecent_gap_event, NULL);
    if (rc != 0)
    {
        MODLOG_DFLT(ERROR, "Error: Failed to connect to device; addr_type=%d "
                           "addr=%s; rc=%d\n",
                    disc->addr.type, addr_str(disc->addr.val), rc);
        return;
    }
}

/**
 * The nimble host executes this callback when a GAP event occurs.  The
 * application associates a GAP event callback with each connection that is
 * established.  blecent uses the same callback for all connections.
 *
 * @param event                 The event being signalled.
 * @param arg                   Application-specified argument; unused by
 *                                  blecent.
 *
 * @return                      0 if the application successfully handled the
 *                                  event; nonzero on failure.  The semantics
 *                                  of the return code is specific to the
 *                                  particular GAP event being signalled.
 */
static int
blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type)
    {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0)
        {
            return 0;
        }

        /* An advertisment report was received during GAP discovery. */
        print_adv_fields(&fields);

        /* Try to connect to the advertiser if it looks interesting. */
        blecent_connect_if_interesting(&event->disc);
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        if (event->connect.status == 0)
        {
            /* Connection successfully established. */
            MODLOG_DFLT(INFO, "Connection established ");

            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
            MODLOG_DFLT(INFO, "\n");

            /* Remember peer. */
            rc = peer_add(event->connect.conn_handle);
            if (rc != 0)
            {
                MODLOG_DFLT(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }

            /* Perform service discovery. */
            rc = peer_disc_all(event->connect.conn_handle,
                               blecent_on_disc_complete, NULL);
            if (rc != 0)
            {
                MODLOG_DFLT(ERROR, "Failed to discover services; rc=%d\n", rc);
                return 0;
            }
        }
        else
        {
            /* Connection attempt failed; resume scanning. */
            MODLOG_DFLT(ERROR, "Error: Connection failed; status=%d\n",
                        event->connect.status);
            blecent_scan();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        /* Connection terminated. */
        MODLOG_DFLT(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        MODLOG_DFLT(INFO, "\n");

        /* Forget about peer. */
        peer_delete(event->disconnect.conn.conn_handle);

        /* Resume scanning. */
        blecent_scan();
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        MODLOG_DFLT(INFO, "discovery complete; reason=%d\n",
                    event->disc_complete.reason);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        /* Encryption has been enabled or disabled for this connection. */
        MODLOG_DFLT(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        /* Peer sent us a notification or indication. */
        MODLOG_DFLT(INFO, "received %s; conn_handle=%d attr_handle=%d "
                          "attr_len=%d\n",
                    event->notify_rx.indication ? "indication" : "notification",
                    event->notify_rx.conn_handle,
                    event->notify_rx.attr_handle,
                    OS_MBUF_PKTLEN(event->notify_rx.om));

        /* Attribute data is contained in event->notify_rx.om. Use
         * `os_mbuf_copydata` to copy the data received in notification mbuf */
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        /* We already have a bond with the peer, but it is attempting to
         * establish a new secure link.  This app sacrifices security for
         * convenience: just throw away the old bond and accept the new link.
         */

        /* Delete the old bond. */
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        /* Return BLE_GAP_REPEAT_PAIRING_RETRY to indicate that the host should
         * continue with the pairing operation.
         */
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

static void
blecent_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
blecent_on_sync(void)
{
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);

    /* Begin scanning for a peripheral to connect to. */
    blecent_scan();
}

void blecent_host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

#define SAMPLE_COUNT UINT16_C(300)

struct bme68x_dev bme;
int8_t result;
struct bme68x_conf conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data data;
uint32_t del_period;
uint32_t time_ms = 0;
uint8_t n_fields;
uint16_t sample_count = 1;

void vtask_bme68x_measurement(void *pv_parameters)
{
    for (;;)
    {
        result = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        bme68x_check_rslt("bme68x_set_op_mode", result);

        /* Calculate delay period in microseconds */
        del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);

        /* Check if rslt == BME68X_OK, report or handle if otherwise */
        result = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        bme68x_check_rslt("bme68x_get_data", result);
    }
}

void vtask_upload_bme68x_measurement(void *pv_parameters)
{
    for (;;)
    {
        cJSON *device = cJSON_CreateObject();
        cJSON *identifiers = cJSON_AddArrayToObject(device, "identifiers");
        cJSON_AddItemToArray(identifiers, cJSON_CreateString("test 1"));
        cJSON_AddStringToObject(device, "manufacturer", "Maarten de Klerk");
        cJSON_AddStringToObject(device, "model", "BLE Central");
        cJSON_AddStringToObject(device, "name", "BLE Central 1");

        cJSON *payload = cJSON_CreateObject();
        cJSON_AddStringToObject(payload, "device_class", "temperature");
        cJSON_AddStringToObject(payload, "state_topic", "homeassistant/sensor/ble_central_1/state");
        cJSON_AddItemToObject(payload, "device", device);
        cJSON_AddNumberToObject(payload, "qos", 1);
        cJSON_AddBoolToObject(payload, "force_update", true);
        cJSON_AddNumberToObject(payload, "expire_after", 1000);
        cJSON_AddStringToObject(payload, "value_template", "{{ value_json.temperature }}");
        cJSON_AddStringToObject(payload, "unit_of_measurement", "°C");
        cJSON_AddStringToObject(payload, "name", "temperature");

        char *payload_string = cJSON_PrintUnformatted(payload);
        esp_mqtt_client_publish(mqtt_client, "homeassistant/sensor/ble_central_1_t/config", payload_string, 0, 1, 0);
        cJSON_Delete(payload);

        device = cJSON_CreateObject();
        identifiers = cJSON_AddArrayToObject(device, "identifiers");
        cJSON_AddItemToArray(identifiers, cJSON_CreateString("test 1"));
        cJSON_AddStringToObject(device, "manufacturer", "Maarten de Klerk");
        cJSON_AddStringToObject(device, "model", "BLE Central");
        cJSON_AddStringToObject(device, "name", "BLE Central 1");

        payload = cJSON_CreateObject();
        cJSON_AddStringToObject(payload, "device_class", "humidity");
        cJSON_AddStringToObject(payload, "state_topic", "homeassistant/sensor/ble_central_1/state");
        cJSON_AddItemToObject(payload, "device", device);
        cJSON_AddNumberToObject(payload, "qos", 1);
        cJSON_AddBoolToObject(payload, "force_update", true);
        cJSON_AddNumberToObject(payload, "expire_after", 1000);
        cJSON_AddStringToObject(payload, "value_template", "{{ value_json.humidity }}");
        cJSON_AddStringToObject(payload, "unit_of_measurement", "%");
        cJSON_AddStringToObject(payload, "name", "humidity");

        payload_string = cJSON_PrintUnformatted(payload);
        esp_mqtt_client_publish(mqtt_client, "homeassistant/sensor/ble_central_1_h/config", payload_string, 0, 1, 0);
        cJSON_Delete(payload);

        payload = cJSON_CreateObject();
        cJSON_AddNumberToObject(payload, "temperature", data.temperature);
        cJSON_AddNumberToObject(payload, "humidity", data.humidity);

        payload_string = cJSON_PrintUnformatted(payload);
        printf(payload_string);
        printf("\n");
        esp_mqtt_client_publish(mqtt_client, "homeassistant/sensor/ble_central_1/state", payload_string, 0, 1, 0);
        cJSON_Delete(payload);

        // wait 10 minutes until next measurement
        vTaskDelay((10 * 60 * 1000) / portTICK_PERIOD_MS);
        // vTaskDelay((1000) / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    bme68x_interface_init(&bme);
    result = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", result);

    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    result = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", result);

    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    result = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", result);

    int rc;
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    esp_mqtt_client_config_t mqtt_config = {
        .uri = "mqtt://192.168.1.245:1883",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    esp_mqtt_client_start(mqtt_client);

    xTaskCreatePinnedToCore(vtask_bme68x_measurement, "bme68x_measurement", 2048, xTaskGetCurrentTaskHandle(), 10, NULL, 1);
    xTaskCreatePinnedToCore(vtask_upload_bme68x_measurement, "upload_bme68x_measurement", 2048, xTaskGetCurrentTaskHandle(), 10, NULL, 0);

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    /* Configure the host. */
    ble_hs_cfg.reset_cb = blecent_on_reset;
    ble_hs_cfg.sync_cb = blecent_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Initialize data structures to track connected peers. */
    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("nimble-blecent");
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();

    nimble_port_freertos_init(blecent_host_task);
}
