/*
 * Copyright (c) 2017 Junhyuk Lee
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * sht31.c
 *
 *  Created on: 2017. 9. 25.
 *      Author: neosarchizo
 */

#include "sht31.h"
#include "math.h"
#include <driver/i2c.h>
#include <esp_log.h>

static char tag[] = "neosarchizo";

esp_err_t sht31_reset() {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_WRITE,
	SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x30, SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0xA2, SHT31_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(SHT31_NUM, cmd,
			1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) {
		return ESP_FAIL;
	}
	return ESP_OK;
}

void sht31_init() {
	int i2c_master_port = SHT31_NUM;
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = SHT31_SDA_IO;
	conf.scl_io_num = SHT31_SCL_IO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = SHT31_FREQ_HZ;
	ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
	ESP_ERROR_CHECK(
			i2c_driver_install(i2c_master_port, conf.mode, SHT31_RX_BUF_DISABLE, SHT31_TX_BUF_DISABLE, 0));
	vTaskDelay(200 / portTICK_PERIOD_MS);

	ESP_ERROR_CHECK(sht31_reset());
	vTaskDelay(10 / portTICK_PERIOD_MS);
}

float sht31_readTemperature() {
//	if (!sht31_readTempHum())
//		return NAN;
	return temp;
}

float sht31_readHumidity() {
//	if (!sht31_readTempHum())
//		return NAN;
	return humidity;
}

bool sht31_readTempHum() {
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_WRITE,
	SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x24, SHT31_ACK_CHECK_EN);
	i2c_master_write_byte(cmd, 0x00, SHT31_ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(SHT31_NUM, cmd,
			1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_FAIL) {
		ESP_LOGD(tag, "0x2400 Failed");
		return false;
	}

	vTaskDelay(500 / portTICK_PERIOD_MS);

	uint8_t readbuffer[6];

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SHT31_ADDRESS << 1) | I2C_MASTER_READ,
	SHT31_ACK_CHECK_EN);

	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 1, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 2, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 3, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 4, SHT31_ACK_VAL));
	ESP_ERROR_CHECK(i2c_master_read_byte(cmd, readbuffer + 5, SHT31_NACK_VAL));

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(SHT31_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (ret == ESP_FAIL) {
		ESP_LOGD(tag, "reading Failed");
		return false;
	}

	ESP_LOGD(tag, "HEX : %x %x %x %x %x %x", readbuffer[0], readbuffer[1],
			readbuffer[2], readbuffer[3], readbuffer[4], readbuffer[5]);

	uint16_t ST, SRH;
	ST = readbuffer[0];
	ST <<= 8;
	ST |= readbuffer[1];

	if (readbuffer[2] != sht31_crc8(readbuffer, 2)) {
		ESP_LOGD(tag, "crc8 : 0 Failed");
		return false;
	}

	SRH = readbuffer[3];
	SRH <<= 8;
	SRH |= readbuffer[4];

	if (readbuffer[5] != sht31_crc8(readbuffer + 3, 2)) {
		ESP_LOGD(tag, "crc8 : 1 Failed");
		return false;
	}

	double stemp = ST;
	stemp *= 175;
	stemp /= 0xffff;
	stemp = -45 + stemp;
	temp = stemp;

	double shum = SRH;
	shum *= 100;
	shum /= 0xFFFF;

	humidity = shum;

	return true;
}

uint8_t sht31_crc8(const uint8_t *data, int len) {
	/*
	 *
	 * CRC-8 formula from page 14 of SHT spec pdf
	 *
	 * Test data 0xBE, 0xEF should yield 0x92
	 *
	 * Initialization data 0xFF
	 * Polynomial 0x31 (x8 + x5 +x4 +1)
	 * Final XOR 0x00
	 */

	const uint8_t POLYNOMIAL = 0x31;
	uint8_t crc = 0xFF;

	for (int j = len; j; --j) {
		crc ^= *data++;

		for (int i = 8; i; --i) {
			crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
		}
	}
	return crc;
}
