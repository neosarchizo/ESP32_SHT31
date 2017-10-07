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
 * sht31.h
 *
 *  Created on: 2017. 9. 25.
 *      Author: neosarchizo
 */

#include <stdbool.h>
#include <esp_err.h>

#ifndef _COMPONENT_SHT31_H_
#define _COMPONENT_SHT31_H_

#define SHT31_NUM 				I2C_NUM_0
#define SHT31_SCL_IO    			18    /*!< gpio number for I2C master clock */
#define SHT31_SDA_IO    			17    /*!< gpio number for I2C master data  */
#define SHT31_FREQ_HZ    		100000     /*!< I2C master clock frequency */
#define SHT31_TX_BUF_DISABLE   	0   /*!< I2C master do not need buffer */
#define SHT31_RX_BUF_DISABLE   	0   /*!< I2C master do not need buffer */
#define SHT31_ADDRESS 			0x44
#define SHT31_SOFTRESET			0x30A2
#define SHT31_ACK_CHECK_EN   	0x1     /*!< I2C master will check ack from slave*/
#define SHT31_ACK_CHECK_DIS  	0x0     /*!< I2C master will not check ack from slave */
#define SHT31_ACK_VAL    0x0         /*!< I2C ack value */
#define SHT31_NACK_VAL   0x1         /*!< I2C nack value */

extern void sht31_init();
extern float sht31_readTemperature();
extern float sht31_readHumidity();
extern bool sht31_readTempHum();
extern esp_err_t sht31_reset();
extern uint8_t sht31_crc8(const uint8_t *data, int len);

float humidity, temp;

#endif /* _COMPONENT_SHT31_H_ */
