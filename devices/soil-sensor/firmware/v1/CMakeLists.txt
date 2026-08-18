cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(blinky)

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
    ${CMAKE_CURRENT_SOURCE_DIR}/soil_sensor_auth_key.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/soil_sensor_auth_key.h
    @ONLY
)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)

target_sources(app PRIVATE 
    fdc1004.c
    fdc1004.h
    main.c
    soil_sensor.c
    soil_sensor.h
)
