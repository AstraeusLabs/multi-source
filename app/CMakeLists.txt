# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fixed_source)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)

set(FREQS 0200;0250;0320;0400;0500;0800;1000;1600;2000;4000)

# 16 KHz samples
foreach(F IN LISTS FREQS)
        generate_inc_file_for_target(app
        src/Sine${F}Hz_1s_16.lc3
        ${gen_dir}/Sine${F}Hz_1s_16.lc3.inc)
endforeach()

# 24 KHz samples
foreach(F IN LISTS FREQS)
        generate_inc_file_for_target(app
        src/Sine${F}Hz_1s_24.lc3
        ${gen_dir}/Sine${F}Hz_1s_24.lc3.inc)
endforeach()

# 48 KHz samples
foreach(F IN LISTS FREQS)
        generate_inc_file_for_target(app
        src/Sine${F}Hz_1s_48.lc3
        ${gen_dir}/Sine${F}Hz_1s_48.lc3.inc)
endforeach()
