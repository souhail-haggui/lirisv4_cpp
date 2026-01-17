/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

extern "C" {
#include <zephyr/kernel.h>
#include <zephyr/stats/stats.h>
#include <zephyr/usb/usb_device.h>

#ifdef CONFIG_MCUMGR_GRP_FS
#include <zephyr/device.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#endif
#ifdef CONFIG_MCUMGR_GRP_STAT
#include <zephyr/mgmt/mcumgr/grp/stat_mgmt/stat_mgmt.h>
#endif

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>

#include <drivers/blink.h>
}

LOG_MODULE_REGISTER(smp_sample);

#include "common.h"

#define STORAGE_PARTITION_LABEL	storage_partition
#define STORAGE_PARTITION_ID	FIXED_PARTITION_ID(STORAGE_PARTITION_LABEL)

/* Define an example stats group; approximates seconds since boot. */
STATS_SECT_START(smp_svr_stats)
STATS_SECT_ENTRY(ticks)
STATS_SECT_END;

/* Assign a name to the `ticks` stat. */
STATS_NAME_START(smp_svr_stats)
STATS_NAME(smp_svr_stats, ticks)
STATS_NAME_END(smp_svr_stats);

/* Define an instance of the stats group. */
STATS_SECT_DECL(smp_svr_stats) smp_svr_stats;

#ifdef CONFIG_MCUMGR_GRP_FS
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(cstorage);
static struct fs_mount_t littlefs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &cstorage,
	.storage_dev = (void *)STORAGE_PARTITION_ID,
	.mnt_point = "/lfs1"
};
#endif

/**
 * BlinkController - RAII class for managing the blink LED device
 * 
 * This class encapsulates LED device initialization and control,
 * following RAII principles for resource management.
 * 
 * Note: Initialization is performed separately from construction to allow
 * for error handling. Device initialization may fail if the hardware is not
 * ready, so we return a boolean from initialize() rather than throwing
 * exceptions (which are disabled in embedded systems).
 */
class BlinkController {
private:
	const struct device *blink_device;
	bool initialized;

public:
	/**
	 * Constructor - initializes member variables but does not initialize device.
	 * Device initialization is deferred to initialize() to allow proper error handling.
	 */
	BlinkController() : blink_device(nullptr), initialized(false) {
	}
	
	/**
	 * Initialize the blink LED device
	 * @return true if successful, false otherwise
	 */
	bool initialize() {
		blink_device = DEVICE_DT_GET(DT_NODELABEL(blink_led));
		if (!device_is_ready(blink_device)) {
			LOG_ERR("Blink LED not ready");
			initialized = false;
			return false;
		}
		initialized = true;
		return true;
	}
	
	/**
	 * Set the LED blink period
	 * @param period_ms Period in milliseconds
	 */
	void setPeriod(uint32_t period_ms) {
		if (initialized && blink_device) {
			blink_set_period_ms(blink_device, period_ms);
		}
	}
	
	/**
	 * Check if the controller is initialized
	 * @return true if initialized, false otherwise
	 */
	bool isInitialized() const {
		return initialized;
	}
};

/**
 * StatsManager - RAII class for managing statistics
 * 
 * This class encapsulates statistics initialization and management.
 */
class StatsManager {
private:
	bool initialized;

public:
	StatsManager() : initialized(false) {
	}
	
	/**
	 * Initialize the statistics system
	 * @return true if successful, false otherwise
	 */
	bool initialize() {
		int rc = STATS_INIT_AND_REG(smp_svr_stats, STATS_SIZE_32, "smp_svr_stats");
		if (rc < 0) {
			LOG_ERR("Error initializing stats system [%d]", rc);
			initialized = false;
			return false;
		}
		initialized = true;
		return true;
	}
	
	/**
	 * Increment the tick counter
	 */
	void incrementTicks() {
		if (initialized) {
			STATS_INC(smp_svr_stats, ticks);
		}
	}
	
	/**
	 * Check if the manager is initialized
	 * @return true if initialized, false otherwise
	 */
	bool isInitialized() const {
		return initialized;
	}
};

#ifdef CONFIG_MCUMGR_GRP_FS
/**
 * FileSystemManager - RAII class for managing filesystem mounting
 * 
 * This class encapsulates filesystem initialization and mounting,
 * following RAII principles.
 */
class FileSystemManager {
private:
	struct fs_mount_t *mount_point;
	bool mounted;

public:
	FileSystemManager(struct fs_mount_t *mnt) : mount_point(mnt), mounted(false) {
	}
	
	/**
	 * Mount the filesystem
	 * @return true if successful, false otherwise
	 */
	bool mount() {
		if (!mount_point) {
			return false;
		}
		
		int rc = fs_mount(mount_point);
		if (rc < 0) {
			LOG_ERR("Error mounting littlefs [%d]", rc);
			mounted = false;
			return false;
		}
		mounted = true;
		return true;
	}
	
	/**
	 * Check if filesystem is mounted
	 * @return true if mounted, false otherwise
	 */
	bool isMounted() const {
		return mounted;
	}
	
	/**
	 * Destructor - unmount filesystem if mounted
	 */
	~FileSystemManager() {
		if (mounted && mount_point) {
			fs_unmount(mount_point);
		}
	}
};
#endif

int main(void)
{
	// Initialize Blink LED Controller
	BlinkController blinkController;
	if (!blinkController.initialize()) {
		return 0;
	}
	blinkController.setPeriod(500);
	
	// Initialize Stats Manager
	// Note: Statistics failures are non-critical, so we continue even if initialization fails
	StatsManager statsManager;
	if (!statsManager.initialize()) {
		LOG_WRN("Statistics initialization failed, continuing without stats");
	}
	
	// Mount filesystem if configured
#ifdef CONFIG_MCUMGR_GRP_FS
	FileSystemManager fsManager(&littlefs_mnt);
	if (!fsManager.mount()) {
		LOG_WRN("Filesystem mounting failed, continuing without filesystem");
	}
#endif

	// Start Bluetooth advertising if configured
#ifdef CONFIG_MCUMGR_TRANSPORT_BT
	start_smp_bluetooth_adverts();
#endif

	/* using __TIME__ ensure that a new binary will be built on every
	 * compile which is convenient when testing firmware upgrade.
	 */
	LOG_INF("build time: " __DATE__ " " __TIME__);

	/* The system work queue handles all incoming mcumgr requests.  Let the
	 * main thread idle while the mcumgr server runs.
	 */
	while (1) {
		k_sleep(K_MSEC(1000));
		statsManager.incrementTicks();
	}
	return 0;
}
