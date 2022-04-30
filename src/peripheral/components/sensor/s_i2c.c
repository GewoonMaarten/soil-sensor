#include "driver/i2c.h"
#include "sensor.h"
#include "esp_log.h"

#define ERR_CHECK(f)                                                                       \
    err = f;                                                                               \
    if (err != ESP_OK)                                                                     \
    {                                                                                      \
        ESP_LOGE(TAG, "Error: %d, __file__: %s, __line__: %d\n", err, __FILE__, __LINE__); \
        goto end;                                                                          \
    }

static const char *TAG = __FILE__;

esp_err_t i2c_controller_init()
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_reg_read_memory(uint8_t reg_addr, uint8_t *data, size_t len, uint8_t delay)
{
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert(handle != NULL);

    ERR_CHECK(i2c_master_start(handle));
    ERR_CHECK(i2c_master_write_byte(handle, HDC1080_ADDRESS << 1 | I2C_MASTER_WRITE, true));
    ERR_CHECK(i2c_master_write_byte(handle, reg_addr, true));

    i2c_master_stop(handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    i2c_cmd_link_delete(handle);

    vTaskDelay(delay / portTICK_PERIOD_MS);

    handle = i2c_cmd_link_create();
    assert(handle != NULL);

    ERR_CHECK(i2c_master_start(handle));
    ERR_CHECK(i2c_master_write_byte(handle, HDC1080_ADDRESS << 1 | I2C_MASTER_READ, true));
    ERR_CHECK(i2c_master_read(handle, data, len, I2C_MASTER_LAST_NACK));

    i2c_master_stop(handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

end:
    i2c_cmd_link_delete(handle);
    return err;
}

esp_err_t i2c_reg_write(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t err = ESP_OK;

    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    assert(handle != NULL);

    ERR_CHECK(i2c_master_start(handle));
    ERR_CHECK(i2c_master_write_byte(handle, HDC1080_ADDRESS << 1 | I2C_MASTER_WRITE, true));
    ERR_CHECK(i2c_master_write_byte(handle, reg_addr, true));
    ERR_CHECK(i2c_master_write(handle, data, len, true));

    i2c_master_stop(handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, handle, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

end:
    i2c_cmd_link_delete(handle);
    return err;
}