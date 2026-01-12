#define DT_DRV_COMPAT microchip_25aa02

#include <zephyr/drivers/eeprom.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#define LOG_LEVEL CONFIG_EEPROM_DEBUG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eeprom);

/* 25AA instruction set */
#define EEPROM_25AA_WRSR  0x01U /* Write STATUS register        */
#define EEPROM_25AA_WRITE 0x02U /* Write data to memory array   */
#define EEPROM_25AA_READ  0x03U /* Read data from memory array  */
#define EEPROM_25AA_WRDI  0x04U /* Reset the write enable latch */
#define EEPROM_25AA_RDSR  0x05U /* Read STATUS register         */
#define EEPROM_25AA_WREN  0x06U /* Set the write enable latch   */

/* 25AA status register bits */
#define EEPROM_25AA_STATUS_WIP BIT(0) /* Write-In-Process   (RO) */
#define EEPROM_25AA_STATUS_WEL BIT(1) /* Write Enable Latch (RO) */
#define EEPROM_25AA_STATUS_BP0 BIT(2) /* Block Protection 0 (RW) */
#define EEPROM_25AA_STATUS_BP1 BIT(3) /* Block Protection 1 (RW) */


struct eeprom_25aa_config {
	struct spi_dt_spec bus;
	size_t size;
	size_t pagesize;
	uint8_t addr_width;
	bool readonly;
	uint16_t timeout;
	bool (*bus_is_ready)(const struct device *dev);
	eeprom_api_read read_fn;
	eeprom_api_write write_fn;
};

struct eeprom_25aa_data {
	struct k_mutex lock;
};

static size_t eeprom_25aa_remaining_len_in_page(const struct device *dev, off_t offset, size_t len)
{
	const struct eeprom_25aa_config *cfg = dev->config;
	off_t page_offset = offset % cfg->pagesize;
	size_t rem = cfg->pagesize - page_offset;

	if (rem > len) {
		rem = len;
	}

	return rem;
}

static int eeprom_25aa_read(const struct device *dev, off_t offset, void *buf,
			    size_t len)
{
	const struct eeprom_25aa_config *config = dev->config;
	struct eeprom_25aa_data *data = dev->data;
	int ret;

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_ERR("Attempt to read past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = config->read_fn(dev, offset, buf, len);
	if (ret < 0) {
		LOG_ERR("Failed to read EEPROM (err %d)", ret);
		k_mutex_unlock(&data->lock);
		return ret;
	}

	k_mutex_unlock(&data->lock);

	return 0;
}

static int eeprom_25aa_write(const struct device *dev, off_t offset,
			     const void *buf,
			     size_t len)
{
	const struct eeprom_25aa_config *config = dev->config;
	struct eeprom_25aa_data *data = dev->data;
	size_t len_in_page;
	uint8_t *pdata = (uint8_t*)buf;
	int ret;

	if (config->readonly) {
		LOG_WRN("attempt to write to read-only device");
		return -EACCES;
	}

	if (!len) {
		return 0;
	}

	if ((offset + len) > config->size) {
		LOG_WRN("attempt to write past device boundary");
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	while (len) {
		len_in_page = eeprom_25aa_remaining_len_in_page(dev, offset, len);
		ret = config->write_fn(dev, offset, pdata, len_in_page);
		if (ret < 0) {
			LOG_ERR("failed to write to EEPROM (err %d)", ret);
			k_mutex_unlock(&data->lock);
			return ret;
		}

		len -= len_in_page;
		pdata += len_in_page;
		offset += len_in_page;
	}

	ret = 0;

	k_mutex_unlock(&data->lock);

	return ret;
}

static size_t eeprom_25aa_size(const struct device *dev)
{
	const struct eeprom_25aa_config *config = dev->config;

	return config->size;
}

static bool eeprom_25AA_bus_is_ready(const struct device *dev)
{
	const struct eeprom_25aa_config *config = dev->config;

	return spi_is_ready_dt(&config->bus);
}

static int eeprom_25AA_rdsr(const struct device *dev, uint8_t *status)
{
	const struct eeprom_25aa_config *config = dev->config;
	uint8_t rdsr = EEPROM_25AA_RDSR;
	uint8_t sr;
	int err;

	const struct spi_buf tx_buf[] = {{
		.buf = &rdsr,
		.len = 1,
	}};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1u,
		},
		{
			.buf = &sr,
			.len = 1u,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	err = spi_transceive_dt(&config->bus, &tx, &rx);
	if (!err) {
		*status = sr;
	}

	return err;
}

static int eeprom_25AA_wait_for_idle(const struct device *dev)
{
	const struct eeprom_25aa_config *config = dev->config;
	int64_t timeout;
	uint8_t status;
	int err;

	timeout = k_uptime_get() + config->timeout;
	while (1) {
		int64_t now = k_uptime_get();
		err = eeprom_25AA_rdsr(dev, &status);
		if (err) {
			LOG_ERR("Could not read status register (err %d)", err);
			return err;
		}
		if (!(status & EEPROM_25AA_STATUS_WIP)) {
			return 0;
		}
		if (now > timeout) {
			break;
		}
		k_sleep(K_MSEC(1));
	}

	return -EBUSY;
}

static int eeprom_25AA_read(const struct device *dev, off_t offset, void *buf,
			    size_t len)
{
	const struct eeprom_25aa_config *config = dev->config;
	struct eeprom_25aa_data *data = dev->data;
	uint8_t cmd[2] = { EEPROM_25AA_READ, offset};
	int err;

	const struct spi_buf tx_buf[] = {{
		.buf = cmd,
		.len = sizeof(cmd),
	}};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = sizeof(cmd),
		},
		{
			.buf = buf,
			.len = len,
		},
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = ARRAY_SIZE(rx_buf),
	};

	err = eeprom_25AA_wait_for_idle(dev);
	if (err) {
		LOG_ERR("EEPROM idle wait failed (err %d)", err);
		k_mutex_unlock(&data->lock);
		return err;
	}

	err = spi_transceive_dt(&config->bus, &tx, &rx);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int eeprom_25AA_wren(const struct device *dev)
{
	const struct eeprom_25aa_config *config = dev->config;
	uint8_t cmd = EEPROM_25AA_WREN;
	const struct spi_buf tx_buf = {
		.buf = &cmd,
		.len = 1,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1,
	};

	return spi_write_dt(&config->bus, &tx);
}

static int eeprom_25AA_write(const struct device *dev, off_t offset,
			     const void *buf, size_t len)
{

	const struct eeprom_25aa_config *config = dev->config;
	uint8_t cmd[2] = {EEPROM_25AA_WRITE, offset};
	int err = 0;

	const struct spi_buf tx_bufs[2] = {
		{
			.buf = (void *)cmd,
			.len = sizeof(cmd),
		},
		{
			.buf = (void *)buf,
			.len = len,
		},
	};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	err = eeprom_25AA_wait_for_idle(dev);
	if (err) {
		LOG_ERR("EEPROM idle wait failed (err %d)", err);
		return err;
	}

	err = eeprom_25AA_wren(dev);
	if (err) {
		LOG_ERR("Failed to disable write protection (err %d)", err);
		return err;
	}

	// err = eeprom_25AA_wait_for_idle(dev);
	// if (err) {
	// 	LOG_ERR("EEPROM idle wait failed (err %d)", err);
	// 	return err;
	// }
	// err = eeprom_25AA_rdsr(dev, &status);
	// if (err) {
	// 	LOG_ERR("Could not read status register (err %d)", err);
	// 	return err;
	// }
	// LOG_HEXDUMP_INF(&status, 1u, "Status after wren:");
	// LOG_HEXDUMP_INF(buf, len, "Data to write:");
	err = spi_write_dt(&config->bus, &tx);
	if (err < 0) {
		return err;
	}

	return 0;
}

static int eeprom_25aa_init(const struct device *dev)
{
	const struct eeprom_25aa_config *config = dev->config;
	struct eeprom_25aa_data *data = dev->data;

	k_mutex_init(&data->lock);

	if (!config->bus_is_ready(dev)) {
		LOG_ERR("Parent bus device not ready");
		return -EINVAL;
	}
	LOG_INF("EEPROM 25AA successfully initialize");

	return 0;
}

static const struct eeprom_driver_api eeprom_25aa_api = {
	.read = eeprom_25aa_read,
	.write = eeprom_25aa_write,
	.size = eeprom_25aa_size,
};

static const struct eeprom_25aa_config eeprom_25aa_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
								    SPI_WORD_SET(8) | SPI_LINES_SINGLE)),
	.size = DT_INST_PROP(0, size),
	.pagesize = DT_INST_PROP(0, pagesize),
	.addr_width = DT_INST_PROP(0, address_width),
	.readonly = DT_INST_PROP(0, read_only),
	.timeout = DT_INST_PROP(0, timeout),
	.bus_is_ready = eeprom_25AA_bus_is_ready,
	.read_fn = eeprom_25AA_read,
	.write_fn = eeprom_25AA_write,
};


static struct eeprom_25aa_data eeprom_25aa_data;


DEVICE_DT_INST_DEFINE(0, eeprom_25aa_init, NULL, &eeprom_25aa_data,
		    &eeprom_25aa_config, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,
		    &eeprom_25aa_api);
