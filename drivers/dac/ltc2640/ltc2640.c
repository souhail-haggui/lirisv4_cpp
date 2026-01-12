/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_ltc2640

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/dac.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ltc2640, CONFIG_LTC2640_DEBUG_LEVEL);

#define LTC2640_OP_WRITE(x)	(x << 4)
#define LTC2640_POR_DELAY      5

enum ltc2640_command {
	LTC2640_WRITE_TO_INPUT = 0,
	LTC2640_UPDATE_DAC_REG = 1,
	LTC2640_WRITE_AND_UPDATE = 3,
	LTC2640_POWER_DOWN = 4,
	LTC2640_SELECT_INT_REF = 6,
	LTC2640_SELECT_EXT_REF = 7
};

struct ltc2640_dev_config {
	struct spi_dt_spec bus;
	uint8_t resolution;
	uint16_t vref;
	uint8_t nchannels;
};

struct ltc2640_data {
	uint16_t intensity;
};

static int ltc2640_write_command(const struct device *dev, enum ltc2640_command command, 
				uint16_t value)
{
	const struct ltc2640_dev_config *config = dev->config;
	uint8_t tx_data[3];
	uint32_t intensity = value;

	tx_data[0] = LTC2640_OP_WRITE(command);
	if (value) {
		intensity <<= config->resolution;
		intensity = intensity / config->vref;
		intensity <<= 4;
	}
	sys_put_be16(intensity, tx_data + 1);
	LOG_DBG("sending to DAC %s command 0x%02X and value 0x%04x", dev->name, tx_data[0], intensity);

	const struct spi_buf buf[1] = {
		{
			.buf = &tx_data,
			.len = sizeof(tx_data)
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 1
	};

	int result = spi_write_dt(&config->bus, &tx);
	if (result != 0) {
		printk("spi_transceive failed with error %i", result);
		return result;
	}

	return 0;
}

static int ltc2640_soft_reset(const struct device *dev)
{
	uint16_t regval = LTC2640_SELECT_INT_REF;
	int ret;

	ret = ltc2640_write_command(dev, regval, 0);
	if (ret) {
		return -EIO;
	}
	k_msleep(LTC2640_POR_DELAY);

	return 0;
}

static int ltc2640_channel_setup(const struct device *dev,
				   const struct dac_channel_cfg *channel_cfg)
{
	const struct ltc2640_dev_config *config = dev->config;
	/* LTC2640 series only has a single output channel. */
	if (channel_cfg->channel_id != 0) {
		LOG_ERR("Unsupported channel %d", channel_cfg->channel_id);
		return -ENOTSUP;
	}

	if (channel_cfg->resolution != config->resolution) {
		LOG_ERR("Unsupported resolution %d. Actual: %d", channel_cfg->resolution,
			config->resolution);
		return -ENOTSUP;
	}

	return 0;
}

static int ltc2640_write_value(const struct device *dev, uint8_t channel,
				uint32_t value)
{
	const struct ltc2640_dev_config *config = dev->config;
	if (channel != 0) {
		LOG_ERR("ltc2640: Unsupported channel %d", channel);
		return -ENOTSUP;
	}

	if (value >= (1 << config->resolution)) {
		LOG_ERR("ltc2640: Value %d out of range", value);
		return -EINVAL;
	}

	return ltc2640_write_command(dev, LTC2640_WRITE_AND_UPDATE, value);
}

static const struct dac_driver_api ltc2640_driver_api = {
	.channel_setup = ltc2640_channel_setup,
	.write_value = ltc2640_write_value,
};

static int ltc2640_init(const struct device *dev)
{
	const struct ltc2640_dev_config *config = dev->config;
	if (!spi_is_ready_dt(&config->bus)) {
		LOG_DBG("spi device not ready: %s", config->bus.bus->name);
		return -EINVAL;
	}

	if (ltc2640_soft_reset(dev)) {
		printk("ltc2640: unable to set internel vref");
		return -EINVAL;
	};

	LOG_INF("DAC LTC2640 successfully initialized");

	return 0;
}

static struct ltc2640_data ltc2640_driver;

static const struct ltc2640_dev_config ltc2640_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | 
	                                SPI_WORD_SET(8) | SPI_LINES_SINGLE), 0),
	.resolution = DT_INST_PROP(0, resolution),
	.vref = DT_INST_PROP(0, voltage_reference_mv),
};

DEVICE_DT_INST_DEFINE(0, ltc2640_init, NULL, &ltc2640_driver,
		    &ltc2640_config, POST_KERNEL, CONFIG_DAC_INIT_PRIORITY,
		    &ltc2640_driver_api);
