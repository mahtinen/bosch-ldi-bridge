idf_component_register(SRCS "bridge.c"
                            "bridge_console.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt nvs_flash esp_timer console
                                cps_server sim_source status_led
                                ldi_client capture ble_link_verify
                                bike_state data_source)
