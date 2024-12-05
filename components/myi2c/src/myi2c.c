/*
 * @Author: mojionghao
 * @Date: 2024-07-29 16:34:41
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-08-12 11:42:26
 * @FilePath: \max30102_test\components\myi2c\src\myi2c.c
 * @Description:
 */

#include "myi2c.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
/**
 * @description: 初始化i2c总线
 * @param {int} i2c_master_port
 * @param {  } i2c_param_config
 * @param {return} i2c_driver_install
 * @return {*}
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM; // 指定I2C设备序号

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 50000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void i2c_scan()
{
    i2c_cmd_handle_t cmd;
    for (int addr = 0; addr < 128; addr++)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        if (err == ESP_OK)
        {
            ESP_LOGI("I2C_SCAN", "Found device at address 0x%02x", addr);
        }
        i2c_cmd_link_delete(cmd);
    }
}