/*
 * @Author: mojionghao
 * @Date: 2024-07-29 16:34:57
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-08-12 11:44:27
 * @FilePath: \max30102_test\components\myi2c\include\myi2c.h
 * @Description:
 */
#pragma once

#include "esp_err.h"

#define I2C_MASTER_SCL_IO GPIO_NUM_4 /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_5 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0             /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000    /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0  /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

extern esp_err_t i2c_master_init(void);
extern void i2c_scan();