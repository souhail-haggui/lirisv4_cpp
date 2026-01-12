/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adt7320

#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "adt7320.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adt7320, CONFIG_ADT7320_DEBUG_LEVEL);

/* Free the CS pin and release the SPI device after a transaction */
#define SPI_OPER_POST_FREE                                                                         \
	(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | SPI_LINES_SINGLE)



static int adt7320_transceive(const struct spi_dt_spec* bus, uint8_t reg,
			      void *data, size_t length)
{
	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1
		}, {
			.buf = data,
			.len = length
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
	};

	if (reg & ADT7320_REG_READ) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		tx.count = 1;

		return spi_transceive_dt(bus, &tx, &rx);
	}

	tx.count = 2;

	return spi_write_dt(bus, &tx);
}

static int adt7320_temp_reg_read(const struct device *dev, uint8_t reg,
				uint8_t *value, uint8_t len)
{
	const struct adt7320_dev_config *cfg = dev->config;
	if (adt7320_transceive(&cfg->bus, ADT7320_OP_READ(reg), value, len) < 0) {
		return -EIO;
	}
	return 0;
}

static int adt7320_temp_reg_write(const struct device *dev, uint8_t reg,
				uint8_t *value, uint8_t len)
{
	const struct adt7320_dev_config *cfg = dev->config;
	if (adt7320_transceive(&cfg->bus, ADT7320_OP_WRITE(reg), value,
								len) < 0) {
		return -EIO;
	}
	return 0;
}

static int adt7320_reset(const struct device *dev)
{
	const struct adt7320_dev_config *cfg = dev->config;
	
	uint8_t data[5] = { 0xFF, 0xF, 0xFF, 0xFF, 0xFF};

	const struct spi_buf buf = {
		.buf = data,
		.len = 5
	};

	struct spi_buf_set tx = {
		.buffers = &buf,
		.count = 1
	};

	return spi_write_dt(&cfg->bus, &tx);
}

static int adt7320_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	int ret;
	struct adt7320_data *drv_data = dev->data;
	uint16_t value;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);

	ret = adt7320_temp_reg_read(dev, ADT7320_REG_TEMP, (uint8_t*)&value,
							  sizeof(uint16_t));
	if (ret < 0) {
		return ret;
	}
	drv_data->sample = sys_be16_to_cpu(value);

	if (drv_data->sample < 0) {
		drv_data->sample = (uint16_t)drv_data->sample - 65536U;
	}
	return 0;
}

static int adt7320_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct adt7320_data *drv_data = dev->data;
	float value;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	value = (float)(drv_data->sample / 128.0f);
	val->val1 = (int32_t) value;
	val->val2 = (int32_t)((float)((value - val->val1) * 1000000));
	return 0;
}

static const struct sensor_driver_api adt7320_driver_api = {
	.sample_fetch = adt7320_sample_fetch,
	.channel_get = adt7320_channel_get,
};

static int adt7320_probe(const struct device *dev)
{
	uint8_t mode = ADT7320_CONFIG_OP_MODE(ADT7320_OP_MODE_1_SPS) |
				ADT7320_CONFIG_RESOLUTION;
	uint8_t value;
	int ret;

	ret = adt7320_temp_reg_read(dev, ADT7320_REG_ID, &value,
				sizeof(uint8_t));
	LOG_DBG("adt7320 ID = %02X", value);
	if (ret) {
		return ret;
	}

	if (value != ADT7320_DEFAULT_ID) {
		return -ENODEV;
	}

	ret = adt7320_temp_reg_write(dev, ADT7320_REG_CONFIG,
			&mode, sizeof(uint8_t));
	if (ret) {
		return ret;
	}
	LOG_INF("Temp adt7320 successfully initialized");

	return 0;
}

static int adt7320_init(const struct device *dev)
{
	const struct adt7320_dev_config *config = dev->config;
	int err;
	
	if (!spi_is_ready_dt(&config->bus)) {
		LOG_DBG("spi device not ready: %s", config->bus.bus->name);
		return -EINVAL;
	}

	err = adt7320_reset(dev);

	if (err) {
		LOG_ERR("adt7320 reset failed, error %d", err);
		return -ENODEV;
	}

	return adt7320_probe(dev);
}

static struct adt7320_data adt7320_driver;

static const struct adt7320_dev_config adt7320_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_OPER_POST_FREE)
};


DEVICE_DT_INST_DEFINE(0, adt7320_init, NULL, &adt7320_driver,
		    &adt7320_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &adt7320_driver_api);
