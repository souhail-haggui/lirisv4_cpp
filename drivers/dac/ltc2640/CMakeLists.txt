#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
zephyr_library()
zephyr_include_directories(.)
zephyr_library_sources_ifdef(CONFIG_LTC2640 ltc2640.c)
