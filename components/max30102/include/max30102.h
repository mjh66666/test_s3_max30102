/*
 * @Author: mojionghao
 * @Date: 2024-08-02 11:37:09
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-08-02 11:50:58
 * @FilePath: \max30102_test\components\max30102\include\max30102.h
 * @Description:
 */
#pragma once

#include <stdio.h>
#include "driver/i2c.h"

#define MAX30102_Device_address 0x57 // 8位地址表示

// register addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

typedef struct
{
    i2c_port_t bus;
    gpio_num_t int_pin;
    uint16_t dev_address;
} max30102_dev_t;

typedef void *max30102_handle_t;

max30102_handle_t max30102_create(i2c_port_t port, const uint16_t dev_addr, gpio_num_t int_pin);

void max30102_delete(max30102_handle_t sensor);

esp_err_t max30102_reset(max30102_handle_t sensor);

esp_err_t max30102_config(max30102_handle_t sensor);

esp_err_t max30102_read_fifo(max30102_handle_t sensor, uint16_t *fifo_red, uint16_t *fifo_ir);

esp_err_t max30102_read_temp(max30102_handle_t sensor, float *temperature);
