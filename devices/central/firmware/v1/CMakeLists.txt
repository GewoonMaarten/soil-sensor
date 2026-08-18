# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

get_filename_component(SOIL_SENSOR_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(EXTRA_COMPONENT_DIRS "${SOIL_SENSOR_REPO_ROOT}/firmware-libs/drivers")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(central)
