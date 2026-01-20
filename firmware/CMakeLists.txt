#
# CMakeLists.txt for LED Copper String Zigbee Controller
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project("LED Copper String Controller")

target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add include path for pm_config.h (needed by partition manager)
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
