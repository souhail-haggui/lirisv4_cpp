#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
zephyr_include_directories(.)
zephyr_library_sources_ifdef(CONFIG_ADT7320 adt7320.c)
