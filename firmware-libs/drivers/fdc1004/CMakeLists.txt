if(COMMAND idf_component_register)
    idf_component_register(
        SRCS "fdc1004.c"
        INCLUDE_DIRS "."
    )
else()
    add_library(fw_fdc1004 STATIC fdc1004.c)
    target_include_directories(fw_fdc1004 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(fw_fdc1004 PRIVATE zephyr_interface)
endif()
