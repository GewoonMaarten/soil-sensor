add_subdirectory(drivers/bme68x)
add_subdirectory(drivers/fdc1004)

if(COMMAND idf_component_register)
    add_subdirectory(drivers/bsec)
endif()
