idf_component_register(
    SRCS "b_advertiser.c" "b_gatt_server.c" "b_utils.c" 
    INCLUDE_DIRS "include"
    PRIV_REQUIRES sensor console bt)
