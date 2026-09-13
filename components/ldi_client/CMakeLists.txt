# Only SRCS is conditional on CONFIG_. REQUIRES deliberately is NOT:
# ESP-IDF processes component CMakeLists in an early pass to build the
# dependency graph, before CONFIG_* values resolve, so a conditional REQUIRES
# silently evaluates empty on a fresh configure and the component's headers
# vanish from the include path -- presenting as a missing header even though
# the symbol is y and the component exists.
if(CONFIG_BRIDGE_ENABLE_LDI)
    set(srcs "ldi_client.c")
else()
    set(srcs "ldi_client_stub.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES bt esp_timer nvs_flash ble_link_verify capture
                                bike_state ldi_proto)
