cmake_minimum_required(VERSION 3.5)
set(COMPONENTS "main retro-go bt nvs_flash")
set(RG_ENABLE_NETWORKING 0)
# Bluetooth is only enabled for this app, on top of the target's sdkconfig
set(SDKCONFIG_DEFAULTS "$ENV{SDKCONFIG_DEFAULTS};${CMAKE_CURRENT_LIST_DIR}/sdkconfig.bluetooth")
include(../base.cmake)
project(printer)
