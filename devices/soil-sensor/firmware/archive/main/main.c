#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
#include "sdkconfig.h"

/* BLE */
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_hs.h"

// #include "sht4x.h"

#include "ble.h"
#include "sensor.h"
#include "config.h"
#include "led.h"

void app_main(void)
{
    int rc;

    adc_init();
    // i2c_controller_init();

    // hdc1080_config_t hdc1080_config = {
    //     .humidity_resolution = HDC1080_CONF_HRES_14BIT,
    //     .temperature_resolution = HDC1080_CONF_TRES_14BIT,
    //     .mode = HDC1080_CONF_MODE_0,
    //     .heater = HDC1080_CONF_HEAT_DIS,
    // };
    // hdc1080_device = hdc1080_init(hdc1080_config);
    moisture_sensor = moisture_init();

    // led_init();
    // led_toggle();

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_nimble_hci_and_controller_init());

    nimble_port_init();
    /* Initialize the NimBLE host configuration */
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name */
    rc = ble_svc_gap_device_name_set(CONFIG_DEVICE_NAME);
    assert(rc == 0);

    /* Start the task */
    nimble_port_freertos_init(ble_host_task);
}
