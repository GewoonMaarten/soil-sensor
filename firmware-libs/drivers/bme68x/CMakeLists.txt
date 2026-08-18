if(COMMAND idf_component_register)
    idf_component_register(
        SRCS "bme68x.c"
        INCLUDE_DIRS "include"
    )
else()
    add_library(fw_bme68x STATIC
        bme68x.c
    )

    target_include_directories(fw_bme68x PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endif()
