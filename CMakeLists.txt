# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Build only the app entry component and the dependencies it declares.
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)


project(zigbee_cct_led_controller)
