/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADT7320_ADT7320_H_
#define ZEPHYR_DRIVERS_SENSOR_ADT7320_ADT7320_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

/* ADT7320 registers */
#define ADT7320_REG_TEMP		0x02 /* Temperature value*/
#define ADT7320_REG_STATUS		0x02 /* Status */
#define ADT7320_REG_CONFIG		    0x01 /* Configuration */
#define ADT7320_REG_T_HIGH_MSB		0x04 /* Temperature HIGH setpoint MSB */
#define ADT7320_REG_T_HIGH_LSB		0x05 /* Temperature HIGH setpoint LSB */
#define ADT7320_REG_T_LOW_MSB		0x06 /* Temperature LOW setpoint MSB */
#define ADT7320_REG_T_LOW_LSB		0x07 /* Temperature LOW setpoint LSB */
#define ADT7320_REG_T_CRIT_MSB		0x08 /* Temperature CRIT setpoint MSB */
#define ADT7320_REG_T_CRIT_LSB		0x09 /* Temperature CRIT setpoint LSB */
#define ADT7320_REG_HIST		0x0A /* Temperature HYST setpoint */
#define ADT7320_REG_ID			0x03 /* ID */
#define ADT7320_REG_RESET		0xFF /* Software reset */

/* Indicates a read operation; bit 6 is clear on write */
#define ADT7320_REG_READ        BIT(6) /* Read from register*/
#define ADT7320_REG_WRITE       0x00   /* Write into register*/

#define ADT7320_OP_READ(x)		((x << 3) | ADT7320_REG_READ) 
#define ADT7320_OP_WRITE(x)     ((x << 3) | ADT7320_REG_WRITE)

/* ADT7320_REG_STATUS definition */
#define ADT7320_STATUS_T_LOW		BIT(4)
#define ADT7320_STATUS_T_HIGH		BIT(5)
#define ADT7320_STATUS_T_CRIT		BIT(6)
#define ADT7320_STATUS_RDY		BIT(7)

/* ADT7320_REG_CONFIG definition */
#define ADT7320_CONFIG_FAULT_QUEUE(x)	((x) & 0x3)
#define ADT7320_CONFIG_CT_POL		BIT(2)
#define ADT7320_CONFIG_INT_POL		BIT(3)
#define ADT7320_CONFIG_INT_CT_MODE	BIT(4)
#define ADT7320_CONFIG_OP_MODE(x)	(((x) & 0x3) << 5)
#define ADT7320_CONFIG_RESOLUTION	BIT(7)

/* ADT7320_CONFIG_FAULT_QUEUE(x) options */
#define ADT7320_FAULT_QUEUE_1_FAULT	0
#define ADT7320_FAULT_QUEUE_2_FAULTS	1
#define ADT7320_FAULT_QUEUE_3_FAULTS	2
#define ADT7320_FAULT_QUEUE_4_FAULTS	3

/* ADT7320_CONFIG_OP_MODE(x) options */
#define ADT7320_OP_MODE_CONT_CONV	0
#define ADT7320_OP_MODE_ONE_SHOT	1
#define ADT7320_OP_MODE_1_SPS		2
#define ADT7320_OP_MODE_SHUTDOWN	3

/* ADT7320 default ID */
#define ADT7320_DEFAULT_ID		0xC3

/* scale in micro degrees Celsius */
#define ADT7320_TEMP_SCALE		0.0078f

struct adt7320_data {
	int16_t sample;
};

struct adt7320_dev_config {
	struct spi_dt_spec bus;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_ADT7320_ADT7320_H_ */
