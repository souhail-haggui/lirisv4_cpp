#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES CXX C)

# Add C++ compiler flags
target_compile_options(app PRIVATE
  $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
)

target_sources(app PRIVATE src/main.cpp)
target_sources_ifdef(CONFIG_MCUMGR_TRANSPORT_BT app PRIVATE src/bluetooth.cpp)