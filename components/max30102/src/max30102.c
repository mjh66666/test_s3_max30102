/*
 * @Author: mojionghao
 * @Date: 2024-08-02 11:37:01
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-10-14 18:39:03
 * @FilePath: \max30102_test\components\max30102\src\max30102.c
 * @Description:
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30102.h"
#include "myi2c.h"
#include "driver/gpio.h"

static const char *max30102 = "max30102";

static esp_err_t max30102_write(max30102_handle_t sensor, uint8_t reg_addr, uint8_t data)
{
	if (sensor == NULL) {
		ESP_LOGE(max30102, "memory null!");
		return ESP_FAIL; // 返回错误代码表示无效参数
	}
	max30102_dev_t *sens = (max30102_dev_t *)sensor;
	if (sens->bus != I2C_NUM_0) {
		ESP_LOGE(max30102, "bus null!");
		return ESP_FAIL; // 返回错误代码表示无效参数
	}
	uint8_t write_buf[2] = {reg_addr, data};
	ESP_LOGI(max30102, "I2C Bus: %d", sens->bus);
	ESP_LOGI(max30102, "Device Address: 0x%02X", sens->dev_address);
	ESP_LOGI(max30102, "Register Address: 0x%02X", reg_addr);
	ESP_LOGI(max30102, "Data: 0x%02X", data);
	ESP_LOGI(max30102, "Buffer Size: %d", (int)sizeof(write_buf));

	return i2c_master_write_to_device(sens->bus, sens->dev_address, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t max30102_read(max30102_handle_t sensor, const uint8_t reg_addr, uint8_t *data, size_t len)
{
	max30102_dev_t *sens = (max30102_dev_t *)sensor;
	return i2c_master_write_read_device(sens->bus, sens->dev_address, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

max30102_handle_t max30102_create(i2c_port_t port, const uint16_t dev_addr, gpio_num_t int_pin)
{
	max30102_dev_t *sensor = (max30102_dev_t *)calloc(1, sizeof(max30102_dev_t));
	if (sensor == NULL) {
		ESP_LOGE(max30102, "Memory allocation failed");
	}
	sensor->bus = port;
	sensor->dev_address = dev_addr;
	sensor->int_pin = int_pin;
	return (max30102_handle_t)sensor;
}

void max30102_delete(max30102_handle_t sensor)
{
	max30102_dev_t *sens = (max30102_dev_t *)sensor;
	free(sens);
}

esp_err_t max30102_reset(max30102_handle_t sensor)
{
	return max30102_write(sensor, REG_MODE_CONFIG, 0X40);
}

static esp_err_t write_sequence(max30102_handle_t sensor, const uint8_t *regs, const uint8_t *values, size_t length)
{
	esp_err_t ret = ESP_OK;
	for (size_t i = 0; i < length; ++i) {
		ret = max30102_write(sensor, regs[i], values[i]);
		if (ret != ESP_OK) {
			return ret;
		}
	}
	return ret;
}

esp_err_t max30102_config(max30102_handle_t sensor)
{
	esp_err_t ret = ESP_OK;
	max30102_dev_t *sens = (max30102_dev_t *)sensor;
	gpio_config_t max30102_int_conf = {};
	max30102_int_conf.intr_type = GPIO_INTR_DISABLE;
	max30102_int_conf.mode = GPIO_MODE_INPUT;
	max30102_int_conf.pin_bit_mask = 1 << (sens->int_pin);
	max30102_int_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	max30102_int_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	ret = gpio_config(&max30102_int_conf);
	if (ret != ESP_OK) {
		return ret;
	}
	

	const uint8_t *REG[] = {
		REG_INTR_ENABLE_1,REG_INTR_ENABLE_2,REG_FIFO_WR_PTR,REG_OVF_COUNTER,REG_FIFO_RD_PTR,
		REG_FIFO_CONFIG,REG_MODE_CONFIG,REG_SPO2_CONFIG,REG_LED1_PA,REG_LED2_PA,REG_PILOT_PA,
		REG_TEMP_CONFIG,
	};
	const uint8_t *data[] = {
		0XC0,0X02,0X00,0X00,0X00,0x0f,
        0x03,0x27,0X32,0X32,0X7F,0X01,
	};

	ret = max30102_reset(sensor);
	if (ret != ESP_OK) {
		return ret;
	}

	// ret = max30102_write(sensor, REG_INTR_ENABLE_1, 0XC0);
	// ret = max30102_write(sensor, REG_INTR_ENABLE_2, 0X02);
	// ret = max30102_write(sensor, REG_FIFO_WR_PTR, 0X00);
	// ret = max30102_write(sensor, REG_OVF_COUNTER, 0X00);
	// ret = max30102_write(sensor, REG_FIFO_RD_PTR, 0X00);
	// ret = max30102_write(sensor, REG_FIFO_CONFIG, 0x0f);
	// ret = max30102_write(sensor, REG_MODE_CONFIG, 0x03);
	// ret = max30102_write(sensor, REG_SPO2_CONFIG, 0x27);
	// ret = max30102_write(sensor, REG_LED1_PA, 0X32);
	// ret = max30102_write(sensor, REG_LED2_PA, 0X32);
	// ret = max30102_write(sensor, REG_PILOT_PA, 0X7F);
	// ret = max30102_write(sensor, REG_TEMP_CONFIG, 0X01);

	return write_sequence(sensor, REG, data, sizeof(REG));
}

esp_err_t max30102_read_fifo(max30102_handle_t sensor, uint16_t *fifo_red, uint16_t *fifo_ir)
{
	uint16_t un_temp;
	uint16_t fifo_temp_red = 0;
	uint16_t fifo_temp_ir = 0;
	uint8_t status_1, status_2;
	uint8_t ach_i2c_data[6];
	esp_err_t ret = ESP_OK;
	ret = max30102_read(sensor, REG_INTR_STATUS_1, &status_1, 1);
	if (ret != ESP_OK) {
		return ret;
	}

	ret = max30102_read(sensor, REG_INTR_STATUS_2, &status_2, 1);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = max30102_read(sensor, REG_FIFO_DATA, ach_i2c_data, 6);
	if (ret != ESP_OK) {
		return ret;
	}
	// rea data
	un_temp = ach_i2c_data[0];
	un_temp <<= 14;
	fifo_temp_red += un_temp;

	un_temp = ach_i2c_data[1];
	un_temp <<= 6;
	fifo_temp_red += un_temp;

	un_temp = ach_i2c_data[2];
	un_temp >>= 2;
	fifo_temp_red += un_temp;

	// ir data
	un_temp = ach_i2c_data[3];
	un_temp <<= 14;
	fifo_temp_ir += un_temp;

	un_temp = ach_i2c_data[4];
	un_temp <<= 6;
	fifo_temp_ir += un_temp;

	un_temp = ach_i2c_data[5];
	un_temp >>= 2;
	fifo_temp_ir += un_temp;

	if (fifo_temp_ir <= 10000) {
		fifo_temp_ir = 0;
	}
	if (fifo_temp_red <= 10000) {
		fifo_temp_red = 0;
	}
	*fifo_red = fifo_temp_red;
	*fifo_ir = fifo_temp_ir;

	return ret;
}

esp_err_t max30102_read_temp(max30102_handle_t sensor, float *temperature)
{
	uint8_t temp_Integer, temp_Frac;
	float temp;
	uint8_t status_1, status_2;
	esp_err_t ret = ESP_OK;
	max30102_dev_t *sens = (max30102_dev_t *)sensor;

	if (gpio_get_level(sens->int_pin) != 0) {
		return ESP_ERR_INVALID_STATE; // 中断状态异常
	}

	ret = max30102_read(sensor, REG_INTR_STATUS_1, &status_1, 1);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = max30102_read(sensor, REG_INTR_STATUS_2, &status_2, 1);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = max30102_read(sensor, REG_TEMP_INTR, &temp_Integer, 1);
	if (ret != ESP_OK) {
		return ret;
	}
	ret = max30102_read(sensor, REG_TEMP_FRAC, &temp_Frac, 1);
	if (ret != ESP_OK) {
		return ret;
	}
	temp = temp_Integer + (float)temp_Frac * 0.0625;
	*temperature = temp;
	return ret;
}
