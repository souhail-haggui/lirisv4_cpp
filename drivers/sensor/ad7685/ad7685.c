/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ad7685

#include <stdio.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ad7685, CONFIG_AD7685_DEBUG_LEVEL);

struct ad7685_data {
	int16_t sample1;
	int16_t sample2;
};

struct ad7685_dev_config {
	struct spi_dt_spec bus;
};

static int ad7685_transceive(const struct spi_dt_spec* bus,
			      void *data, size_t length)
{
	const struct spi_buf buf[1] = {
		{
			.buf = data,
			.len = length
		}
	};
	const struct spi_buf_set rx = {
		.buffers = buf,
		.count = 1
	};

	return spi_read_dt(bus, &rx);
}

static int ad7685_read(const struct device *dev, uint8_t *value,
					   uint8_t len)
{
	const struct ad7685_dev_config *cfg = dev->config;
	if (ad7685_transceive(&cfg->bus, value, len) < 0) {
		return -EIO;
	}
	return 0;
}

static int ad7685_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	int ret;
	struct ad7685_data *drv_data = dev->data;
	uint8_t rx_buff[4] = {0};

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_AMBIENT_TEMP);

	ret = ad7685_read(dev,(uint8_t*)rx_buff, sizeof(rx_buff));
	if (ret < 0) {
		return ret;
	}
	drv_data->sample1 =
			0xFFFF - (((uint16_t)rx_buff[0] << 8) | rx_buff[1]);
	drv_data->sample2 =
			0xFFFF - (((uint16_t)rx_buff[2] << 8) | rx_buff[3]);
	return 0;
}

static int ad7685_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct ad7685_data *drv_data = dev->data;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	val->val1 = (int32_t)drv_data->sample1;
	val->val2 = (int32_t)drv_data->sample2;
	return 0;
}

static const struct sensor_driver_api ad7685_driver_api = {
	.sample_fetch = ad7685_sample_fetch,
	.channel_get = ad7685_channel_get,
};


static int ad7685_init(const struct device *dev)
{
	const struct ad7685_dev_config *config = dev->config;
	
	if (!spi_is_ready_dt(&config->bus)) {
		LOG_DBG("spi device not ready: %s", config->bus.bus->name);
		return -EINVAL;
	}

	LOG_INF("ADC ad7685 successfully initialized");

	return 0;
}

static struct ad7685_data ad7685_driver;

static const struct ad7685_dev_config ad7685_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
									SPI_MODE_CPOL | SPI_MODE_CPHA |
								    SPI_WORD_SET(8) | SPI_LINES_SINGLE))};

DEVICE_DT_INST_DEFINE(0, ad7685_init, NULL, &ad7685_driver,
		    &ad7685_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &ad7685_driver_api);
