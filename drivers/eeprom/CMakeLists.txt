#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Only create the library and add sources if the specific EEPROM is enabled
if(CONFIG_EEPROM_25AAA)
  zephyr_library()
  zephyr_include_directories(.)
  zephyr_library_sources(eeprom_25aa02.c)
endif()
