#include "ldi_decode.h"
#include "pb_scan.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ldi_dec";

/*
 * Field numbers, from the LDI specification v1.0 section 2.4
 * (transcribed in proto/ldi.proto).
 */
#define F_SPEED               1  /* uint32, 1/100 km/h                 */
#define F_CADENCE             2  /* int32,  1 rpm                      */
#define F_RIDER_POWER         5  /* uint32, 1 W                        */
#define F_AMBIENT_BRIGHTNESS  9  /* uint32, 1/1000 lux                 */
#define F_BATTERY_SOC        10  /* uint32, 1 %                        */
#define F_TIME               11  /* int64,  seconds since epoch        */
#define F_ODOMETER           12  /* uint32, 1 m                        */
#define F_BIKE_LIGHT         17  /* enum LightState                    */
#define F_SYSTEM_LOCKED      21  /* bool                               */
#define F_CHARGER_CONNECTED  22  /* bool                               */
#define F_LIGHT_RESERVE      23  /* bool                               */
#define F_DIAGNOSIS_ACTIVE   24  /* bool                               */
#define F_BIKE_NOT_DRIVING   25  /* bool                               */

esp_err_t ldi_decode_frame(const uint8_t *buf, size_t len,
                           bike_state_t *state, uint32_t *out_fields)
{
    if (buf == NULL || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    pb_scan_t s;
    pb_field_t f;
    uint32_t touched = 0;
    unsigned unknown = 0;
    const int64_t now = esp_timer_get_time();

    pb_scan_init(&s, buf, len);

    /*
     * MERGE into the caller's state; never clear it first.
     *
     * Notifications are partial: spec 2.2.4.3 says a field is omitted when
     * unchanged, and LDI-002 records that which unchanged fields get included
     * anyway is not guaranteed. So absence means "unchanged", not "zero", and
     * overwriting the whole struct per frame would blank most of it every
     * second.
     */
    while (pb_scan_next(&s, &f)) {
        switch (f.field) {

        case F_SPEED:
            /* 1/100 km/h -> m/s.  (km/h) / 3.6 = m/s, so /100/3.6 = /360. */
            state->speed_mps = (float)f.varint / 360.0f;
            bike_state_mark(state, BIKE_F_SPEED, now);
            touched |= 1u << BIKE_F_SPEED;
            break;

        case F_CADENCE: {
            /* int32 in the schema, and negative is representable. Clamp: a
             * negative cadence has no meaning for the CPS crank model, which
             * treats <= 0.5 as "not pedalling". */
            int32_t rpm = pb_as_int32(f.varint);
            state->cadence_rpm = (rpm > 0) ? (uint16_t)rpm : 0;
            bike_state_mark(state, BIKE_F_CADENCE, now);
            touched |= 1u << BIKE_F_CADENCE;
            break;
        }

        case F_RIDER_POWER:
            /*
             * Field 5, not field 7.  Field 7 also carries a power-shaped value
             * (probably motor power) but is absent from the specification, and
             * 2.2.4.3 requires unrecognised fields to be ignored. Using it
             * would record motor output as the rider's effort and inflate
             * every training metric downstream.
             */
            state->rider_power_w = (f.varint > 65535) ? 65535
                                                      : (uint16_t)f.varint;
            bike_state_mark(state, BIKE_F_RIDER_POWER, now);
            touched |= 1u << BIKE_F_RIDER_POWER;
            break;

        case F_AMBIENT_BRIGHTNESS:
            /* 1/1000 lux; bike_state carries whole lux. */
            state->ambient_brightness =
                (uint16_t)((f.varint / 1000u > 65535u) ? 65535u
                                                       : f.varint / 1000u);
            bike_state_mark(state, BIKE_F_AMBIENT_BRIGHTNESS, now);
            touched |= 1u << BIKE_F_AMBIENT_BRIGHTNESS;
            break;

        case F_BATTERY_SOC:
            state->battery_soc_pct = (f.varint > 100) ? 100 : (uint8_t)f.varint;
            bike_state_mark(state, BIKE_F_BATTERY_SOC, now);
            touched |= 1u << BIKE_F_BATTERY_SOC;
            break;

        case F_TIME:
            state->bike_time_s = (uint32_t)pb_as_int64(f.varint);
            bike_state_mark(state, BIKE_F_TIME, now);
            touched |= 1u << BIKE_F_TIME;
            break;

        case F_ODOMETER:
            state->odometer_m = (uint32_t)f.varint;
            bike_state_mark(state, BIKE_F_ODOMETER, now);
            touched |= 1u << BIKE_F_ODOMETER;
            break;

        case F_BIKE_LIGHT:
            /* LightState: 0 INVALID, 1 OFF, 2 ON. */
            state->light_status = (uint8_t)f.varint;
            bike_state_mark(state, BIKE_F_LIGHT_STATUS, now);
            touched |= 1u << BIKE_F_LIGHT_STATUS;
            break;

        case F_SYSTEM_LOCKED:
            state->system_locked = (f.varint != 0);
            bike_state_mark(state, BIKE_F_SYSTEM_LOCK, now);
            touched |= 1u << BIKE_F_SYSTEM_LOCK;
            break;

        case F_CHARGER_CONNECTED:
            state->charger_connected = (f.varint != 0);
            bike_state_mark(state, BIKE_F_CHARGER_CONNECTED, now);
            touched |= 1u << BIKE_F_CHARGER_CONNECTED;
            break;

        case F_LIGHT_RESERVE:
            state->light_reserve_pct = (f.varint != 0) ? 1 : 0;
            bike_state_mark(state, BIKE_F_LIGHT_RESERVE, now);
            touched |= 1u << BIKE_F_LIGHT_RESERVE;
            break;

        case F_DIAGNOSIS_ACTIVE:
            state->diagnosis_connected = (f.varint != 0);
            bike_state_mark(state, BIKE_F_DIAGNOSIS_CONNECTED, now);
            touched |= 1u << BIKE_F_DIAGNOSIS_CONNECTED;
            break;

        case F_BIKE_NOT_DRIVING:
            state->standstill = (f.varint != 0);
            bike_state_mark(state, BIKE_F_STANDSTILL, now);
            touched |= 1u << BIKE_F_STANDSTILL;
            break;

        default:
            /*
             * Required by 2.2.4.3, and load-bearing: this firmware has already
             * observed active undocumented fields (7, 13-16, 18-20, 27) on a
             * bike whose spec revision does not mention them.
             */
            unknown++;
            break;
        }
    }

    if (out_fields) {
        *out_fields = touched;
    }

    if (touched == 0) {
        ESP_LOGW(TAG, "frame of %u bytes yielded no known fields", (unsigned)len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    ESP_LOGD(TAG, "decoded %u known field(s), ignored %u unknown",
             (unsigned)__builtin_popcount(touched), unknown);
    return ESP_OK;
}

const char *ldi_light_state_name(uint8_t v)
{
    switch (v) {
    case 0:  return "invalid";
    case 1:  return "off";
    case 2:  return "on";
    default: return "?";
    }
}
