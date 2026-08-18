if(COMMAND idf_component_register)
    idf_component_register(
        SRCS "bsec.c"
        INCLUDE_DIRS "include"
        PRIV_REQUIRES "bme68x"
    )

    add_prebuilt_library(algobsec "libalgobsec.a")
    target_link_libraries(${COMPONENT_LIB} PRIVATE algobsec)
else()
    add_library(fw_bsec STATIC
        bsec.c
    )

    target_include_directories(fw_bsec PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    target_link_libraries(fw_bsec PUBLIC fw_bme68x)

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/libalgobsec.a")
        target_link_libraries(fw_bsec PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/libalgobsec.a
        )
    endif()
endif()