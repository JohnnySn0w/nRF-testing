# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blinky)

target_sources(app PRIVATE src/main.c src/callbacks.c src/vcnl4040.c src/gpioDev.c src/flags.c src/remoteService/remote.c)

zephyr_library_include_directories(src/remoteService)
