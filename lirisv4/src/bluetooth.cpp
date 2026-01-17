/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

extern "C" {
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
}

LOG_MODULE_REGISTER(smp_bt_sample);

// Bluetooth advertisement data
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/**
 * BluetoothManager - Singleton class for managing Bluetooth functionality
 * 
 * This class encapsulates all Bluetooth operations including advertising,
 * connection management, and callbacks. The Singleton pattern ensures
 * only one instance manages the Bluetooth state.
 */
class BluetoothManager {
private:
	struct k_work advertise_work;
	
	// Forward declaration of static callbacks
	static void advertiseCallback(struct k_work *work);
	
	// Private constructor for Singleton pattern
	BluetoothManager() {
		k_work_init(&advertise_work, advertiseCallback);
	}
	
	// Delete copy constructor and assignment operator
	BluetoothManager(const BluetoothManager&) = delete;
	BluetoothManager& operator=(const BluetoothManager&) = delete;
	
	/**
	 * Internal method to start advertising
	 */
	void advertise() {
		int rc = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
		if (rc) {
			LOG_ERR("Advertising failed to start (rc %d)", rc);
			return;
		}
		LOG_INF("Advertising successfully started");
	}
	
	/**
	 * Internal method called when a connection is established
	 */
	void onConnected(struct bt_conn *conn, uint8_t err) {
		if (err) {
			LOG_ERR("Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
			k_work_submit(&advertise_work);
		} else {
			LOG_INF("Connected");
		}
	}
	
	/**
	 * Internal method called when a connection is disconnected
	 */
	void onDisconnected(struct bt_conn *conn, uint8_t reason) {
		LOG_INF("Disconnected, reason 0x%02x %s", reason, bt_hci_err_to_str(reason));
	}
	
	/**
	 * Internal method called when a connection is recycled
	 */
	void onConnRecycled() {
		k_work_submit(&advertise_work);
	}
	
	/**
	 * Internal method called when Bluetooth is ready
	 */
	void onBluetoothReady(int err) {
		if (err != 0) {
			LOG_ERR("Bluetooth failed to initialise: %d", err);
		} else {
			k_work_submit(&advertise_work);
		}
	}

public:
	/**
	 * Get the singleton instance
	 * Returns a reference to the single BluetoothManager instance.
	 * Uses a static local variable (Meyer's Singleton) to ensure
	 * thread-safe initialization and proper lifetime management.
	 */
	static BluetoothManager& getInstance() {
		static BluetoothManager instance;
		return instance;
	}
	
	/**
	 * Start Bluetooth advertising
	 */
	void start() {
		int rc = bt_enable(btReadyCallback);
		if (rc != 0) {
			LOG_ERR("Bluetooth enable failed: %d", rc);
		}
	}
	
	// Static callbacks for C API - these redirect to member functions
	static void advertiseCallback(struct k_work *work) {
		getInstance().advertise();
	}
	
	static void connectedCallback(struct bt_conn *conn, uint8_t err) {
		getInstance().onConnected(conn, err);
	}
	
	static void disconnectedCallback(struct bt_conn *conn, uint8_t reason) {
		getInstance().onDisconnected(conn, reason);
	}
	
	static void recycledCallback(void) {
		getInstance().onConnRecycled();
	}
	
	static void btReadyCallback(int err) {
		getInstance().onBluetoothReady(err);
	}
};

// Define connection callbacks using static methods
BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = BluetoothManager::connectedCallback,
	.disconnected = BluetoothManager::disconnectedCallback,
	.recycled = BluetoothManager::recycledCallback,
};

// C-compatible interface function
extern "C" void start_smp_bluetooth_adverts(void)
{
	BluetoothManager::getInstance().start();
}
