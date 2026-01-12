#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
zephyr_include_directories(.)
zephyr_library_sources_ifdef(CONFIG_AD7685 ad7685.c)
