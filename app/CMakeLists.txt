# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(fixed_source)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)

generate_inc_file_for_target(app
src/Sine1000Hz_1s_16.lc3
${gen_dir}/Sine1000Hz_1s_16.lc3.inc)

generate_inc_file_for_target(app
src/Sine1500Hz_1s_16.lc3
${gen_dir}/Sine1500Hz_1s_16.lc3.inc)

generate_inc_file_for_target(app
src/Sine1000Hz_1s_24.lc3
${gen_dir}/Sine1000Hz_1s_24.lc3.inc)

generate_inc_file_for_target(app
src/Sine1500Hz_1s_24.lc3
${gen_dir}/Sine1500Hz_1s_24.lc3.inc)
