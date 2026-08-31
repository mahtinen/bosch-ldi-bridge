/*
 * ldi_decode -- turn a Bosch LDI protobuf notification into bike_state_t.
 *
 * The only place LDI units are converted. Everything downstream sees the
 * canonical units declared in bike_state.h, so when a unit turns out to be
 * different from what was assumed, exactly one line here changes.
 *
 * Field numbers and units come from the LDI specification v1.0 section 2.4,
 * transcribed in proto/ldi.proto.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "bike_state.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Decode one notification, MERGING into *state.
 *
 * Merging is required, not an optimisation: spec 2.2.4.3 omits unchanged
 * fields, so absence means "unchanged" and clearing the struct per frame would
 * blank most of it every second.
 *
 * out_fields, if non-NULL, receives a bitmask of the bike_field_t values this
 * frame actually carried -- useful for logging what moved.
 *
 * Returns ESP_ERR_INVALID_RESPONSE if the frame contained no recognised field,
 * which is the signature of a truncated or corrupted payload (see LDI-003).
 */
esp_err_t ldi_decode_frame(const uint8_t *buf, size_t len,
                           bike_state_t *state, uint32_t *out_fields);

/* LightState enum (field 17) as text. */
const char *ldi_light_state_name(uint8_t v);

#ifdef __cplusplus
}
#endif
