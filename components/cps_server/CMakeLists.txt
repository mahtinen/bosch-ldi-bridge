idf_component_register(SRCS "cps_server.c"
                            "cps_gatt.c"
                            "cps_ctrl_point.c"
                            "crank_model.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bt esp_timer capture esp_app_format)
