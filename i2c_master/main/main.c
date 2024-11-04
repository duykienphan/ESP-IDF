#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#define I2C_SLAVE_ADDR 0x32
#define RX_BUFFER_LEN 255
#define TX_BUFFER_LEN 255
#define TIMEOUT_MS 1000
#define DELAY_MS 3000

#define LED_PIN 2

static const char *TAG = "MASTER";

static esp_err_t set_i2c(void)
{
    i2c_config_t i2c_config = {};

    i2c_config.mode = I2C_MODE_MASTER;
    i2c_config.sda_io_num = 21;
    i2c_config.scl_io_num = 22;
    i2c_config.sda_pullup_en = true;
    i2c_config.scl_pullup_en = true;
    i2c_config.master.clk_speed = 400000;
    i2c_config.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    return ESP_OK;
}

esp_err_t i2c_master_write_device(i2c_port_t i2c_num, uint8_t address, const uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t *)data, size, true);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(TIMEOUT_MS));

    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_master_read_device(i2c_port_t i2c_num, uint8_t address, uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK); // Read all but the last byte
    }
    i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK); // Read the last byte
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(TIMEOUT_MS));

    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{
    uint8_t rx_data[RX_BUFFER_LEN];
    char sendata[TX_BUFFER_LEN];
    int i = 0;

    ESP_ERROR_CHECK(set_i2c());

    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1)
    {
        i++;
        snprintf(sendata, TX_BUFFER_LEN, "MASTER %d\n", i);

        // Clear rx_data buffer before receiving new data
        memset(rx_data, 0, RX_BUFFER_LEN);

        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));

        // Write data to the slave device
        esp_err_t write_ret = i2c_master_write_device(I2C_NUM_0, 
                                                    I2C_SLAVE_ADDR, 
                                                    (uint8_t *)sendata, 
                                                    strlen(sendata));
        if (write_ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Data sent: %s", sendata);
        }
        else
        {
            ESP_LOGE(TAG, "I2C write error: %s", esp_err_to_name(write_ret));
        }

        // Read data from the slave device
        esp_err_t read_ret = i2c_master_read_device(I2C_NUM_0, 
                                                    I2C_SLAVE_ADDR, 
                                                    rx_data, 
                                                    RX_BUFFER_LEN);
        if (read_ret == ESP_OK)
        {
            ESP_LOGI(TAG, "Received data: %s", (char *)rx_data);
        }
        else
        {
            ESP_LOGE(TAG, "I2C read error: %s", esp_err_to_name(read_ret));
        }


        if(strncmp((const char*)rx_data, "LED ON", 6) == 0)
        {   
            gpio_set_level(LED_PIN, 1);
        }
        else if(strncmp((const char*)rx_data, "LED OFF", 7) == 0)
        {
            gpio_set_level(LED_PIN, 0);
        }
    }
}
