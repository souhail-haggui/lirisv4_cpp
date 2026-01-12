#
# Copyright (c) 2019 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

# Only create the library and add sources if the specific FRAM is enabled
if(CONFIG_FRAM_CY15B104Q)
  zephyr_library()
  zephyr_include_directories(.)
  zephyr_library_sources(cy15b104q.c)
endif()
