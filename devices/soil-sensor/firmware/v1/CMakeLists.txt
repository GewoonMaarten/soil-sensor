cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(blinky)

get_filename_component(SOIL_SENSOR_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." ABSOLUTE)
set(SOIL_SENSOR_FIRMWARE_LIBS_DIR "${SOIL_SENSOR_REPO_ROOT}/firmware-libs")
add_subdirectory(
    ${SOIL_SENSOR_FIRMWARE_LIBS_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/firmware-libs
)

set(SOIL_SENSOR_DEFAULT_AUTH_KEY_HEX "2a5df318c74091e653ac743910bd86cf")
set(SOIL_SENSOR_AUTH_KEY_HEX "$ENV{SOIL_SENSOR_AUTH_KEY_HEX}" CACHE STRING
    "16-byte HMAC key as 32 hex chars")

string(STRIP "${SOIL_SENSOR_AUTH_KEY_HEX}" SOIL_SENSOR_AUTH_KEY_HEX)

if("${SOIL_SENSOR_AUTH_KEY_HEX}" STREQUAL "")
    set(SOIL_SENSOR_AUTH_KEY_HEX "${SOIL_SENSOR_DEFAULT_AUTH_KEY_HEX}")
    message(WARNING "SOIL_SENSOR_AUTH_KEY_HEX not set; using development key")
endif()

string(LENGTH "${SOIL_SENSOR_AUTH_KEY_HEX}" SOIL_SENSOR_AUTH_KEY_HEX_LEN)
if(NOT SOIL_SENSOR_AUTH_KEY_HEX_LEN EQUAL 32)
    message(FATAL_ERROR "SOIL_SENSOR_AUTH_KEY_HEX must contain exactly 32 hex characters")
endif()

if(NOT "${SOIL_SENSOR_AUTH_KEY_HEX}" MATCHES "^[0-9A-Fa-f]+$")
    message(FATAL_ERROR "SOIL_SENSOR_AUTH_KEY_HEX must contain only hex characters")
endif()

string(REGEX MATCHALL ".." SOIL_SENSOR_AUTH_KEY_BYTE_LIST "${SOIL_SENSOR_AUTH_KEY_HEX}")
set(SOIL_SENSOR_AUTH_KEY_INITIALIZER "")
foreach(key_byte IN LISTS SOIL_SENSOR_AUTH_KEY_BYTE_LIST)
    string(APPEND SOIL_SENSOR_AUTH_KEY_INITIALIZER "0x${key_byte}, ")
endforeach()

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/soil_sensor_auth_key.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/soil_sensor_auth_key.h
    @ONLY
)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_sources(app PRIVATE 
    src/main.c
    src/soil_sensor.c
)

target_link_libraries(app PRIVATE fw_fdc1004)
