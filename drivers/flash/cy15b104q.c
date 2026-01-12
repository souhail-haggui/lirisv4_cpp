/*
 * Zephyr driver for Cypress CY15B104Q (512Kbit SPI FRAM)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_FRAM_CY15B104Q_DEBUG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(cy15b104q);

#define DT_DRV_COMPAT cypress_cy15b104q

#if DT_INST_NODE_HAS_PROP(0, size)
#define INST_0_BYTES (DT_INST_PROP(0, size) / 8)
#else
#error "No size specified. 'size' or 'size-in-bytes' must be set"
#endif

#define FRAM_CMD_WREN    0x06
#define FRAM_CMD_WRDI    0x04
#define FRAM_CMD_RDSR    0x05
#define FRAM_CMD_WRSR    0x01
#define FRAM_CMD_READ    0x03
#define FRAM_CMD_WRITE   0x02

struct cy15b104q_config {
    struct spi_dt_spec bus;
};

struct cy15b104q_data {
    struct k_mutex lock;
};

static int cy15b104q_write_enable(const struct device *dev)
{
    const struct cy15b104q_config *cfg = dev->config;
    uint8_t cmd = FRAM_CMD_WREN;

    const struct spi_buf buf = {
        .buf = &cmd,
        .len = 1,
    };
    const struct spi_buf_set tx = {
        .buffers = &buf,
        .count = 1,
    };
    return spi_write_dt(&cfg->bus, &tx);
}

static int cy15b104q_read(const struct device *dev, off_t offset,
        void *data, size_t len)
{
    const struct cy15b104q_config *cfg = dev->config;
    struct cy15b104q_data *drv_data = dev->data;

    k_mutex_lock(&drv_data->lock, K_FOREVER);

    uint8_t cmd[4];
    cmd[0] = FRAM_CMD_READ;
    cmd[1] = (offset >> 16) & 0xFF;
    cmd[2] = (offset >> 8) & 0xFF;
    cmd[3] = offset & 0xFF;

    struct spi_buf tx_bufs[2] = {
        { .buf = cmd, .len = 4 },
        { .buf = NULL, .len = 0 }
    };
    struct spi_buf rx_bufs[2] = {
        { .buf = NULL, .len = 4 },
        { .buf = data, .len = len }
    };

    const struct spi_buf_set tx_set = { tx_bufs, 2 };
    const struct spi_buf_set rx_set = { rx_bufs, 2 };

    int ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
    
    k_mutex_unlock(&drv_data->lock);
    
    if (ret < 0) {
        LOG_ERR("FRAM read failed at offset 0x%lx, len %zu: %d", (long)offset, len, ret);
    }
    
    return ret;
}

static int cy15b104q_write(const struct device *dev, off_t offset,
        const void *data, size_t len)
{
    const struct cy15b104q_config *cfg = dev->config;
    struct cy15b104q_data *drv_data = dev->data;

    k_mutex_lock(&drv_data->lock, K_FOREVER);

    int ret = cy15b104q_write_enable(dev);
    if (ret < 0) {
        k_mutex_unlock(&drv_data->lock);
        LOG_ERR("FRAM write enable failed: %d", ret);
        return ret;
    }

    uint8_t cmd[4];
    cmd[0] = FRAM_CMD_WRITE;
    cmd[1] = (offset >> 16) & 0xFF;
    cmd[2] = (offset >> 8) & 0xFF;
    cmd[3] = offset & 0xFF;

    struct spi_buf tx_bufs[2] = {
        { .buf = cmd, .len = 4 },
        { .buf = (void *)data, .len = len },
    };
    const struct spi_buf_set tx = { tx_bufs, 2 };

    ret = spi_write_dt(&cfg->bus, &tx);
    
    k_mutex_unlock(&drv_data->lock);
    
    if (ret < 0) {
        LOG_ERR("FRAM write failed at offset 0x%lx, len %zu: %d", (long)offset, len, ret);
    }
    
    return ret;
}

static int cy15b104q_erase(const struct device *dev, off_t offset, size_t size)
{
    /* FRAM does not require erase before write */
    return 0;
}

static const struct flash_parameters *
cy15b104q_get_parameters(const struct device *dev)
{
    ARG_UNUSED(dev);
    static const struct flash_parameters cy15b104q_flash_parameters = {
        .write_block_size = 1,
        .erase_value = 0xFF,
    };

    return &cy15b104q_flash_parameters;
}

int cy15b104q_get_size(const struct device *dev, uint64_t *size)
{
	ARG_UNUSED(dev);

	*size = (uint64_t)(INST_0_BYTES);

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void cy15b104q_pages_layout(const struct device *dev,
                                    const struct flash_pages_layout **layout,
                                    size_t *layout_size)
{
	/* FRAM doesn't have physical pages, but we need to report a logical page size
	 * Use 4KB pages which is common for flash filesystems */
	static struct flash_pages_layout cy15b104q_layout = {
		.pages_count = INST_0_BYTES / 4096,  /* Number of 4KB pages */
		.pages_size = 4096,                    /* 4KB per page */
	};

	*layout = &cy15b104q_layout;
	*layout_size = 1;
}
#endif

static int cy15b104q_init(const struct device *dev)
{
    const struct cy15b104q_config *cfg = dev->config;
    struct cy15b104q_data *data = dev->data;

    k_mutex_init(&data->lock);

    if (!spi_is_ready_dt(&cfg->bus)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    LOG_INF("Cypress CY15B104Q FRAM initialized");
    return 0;
}

static const struct flash_driver_api cy15b104q_api = {
    .read = cy15b104q_read,
    .write = cy15b104q_write,
    .erase = cy15b104q_erase,
    .get_parameters = cy15b104q_get_parameters,
    .get_size = cy15b104q_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
    .page_layout = cy15b104q_pages_layout,
#endif
};

#define CY15B104Q_INIT(n) \
    static const struct cy15b104q_config cy15b104q_cfg_##n = { \
        .bus = SPI_DT_SPEC_INST_GET(n, SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | \
                                       SPI_TRANSFER_MSB | SPI_LINES_SINGLE), \
    }; \
    static struct cy15b104q_data cy15b104q_data_##n; \
    DEVICE_DT_INST_DEFINE(n, cy15b104q_init, NULL, \
        &cy15b104q_data_##n, &cy15b104q_cfg_##n, \
        POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY, &cy15b104q_api);

DT_INST_FOREACH_STATUS_OKAY(CY15B104Q_INIT)
