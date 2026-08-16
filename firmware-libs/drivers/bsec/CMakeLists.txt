idf_component_register(
    SRCS "bsec.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES " bme68x")

add_prebuilt_library(algobsec "libalgobsec.a")
target_link_libraries(${COMPONENT_LIB} PRIVATE algobsec)