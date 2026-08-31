# No protobuf library dependency: ldi_decode.c walks the wire format directly
# via pb_scan.c. That is both dependency-free and exactly the tolerance the
# spec asks for -- section 2.2.4.3 requires clients to ignore unrecognised
# fields, which a generated parser resists.
#
# protobuf-c 1.4.1 ships inside ESP-IDF if generated code is ever wanted;
# proto/ldi.proto is kept for that, and for reference.
idf_component_register(SRCS "pb_scan.c" "ldi_decode.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bike_state esp_timer)
