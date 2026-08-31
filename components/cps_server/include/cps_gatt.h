/*
 * cps_gatt -- the Cycling Power Service GATT database and the byte-level
 * packing of the CP Measurement characteristic.
 *
 * All multi-byte values are little-endian (the GATT default).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Assigned numbers ------------------------------------------------ */
#define CPS_SVC_UUID16                  0x1818 /* Cycling Power Service    */
#define CPS_CHR_MEASUREMENT_UUID16      0x2A63 /* notify only              */
#define CPS_CHR_FEATURE_UUID16          0x2A65 /* read, uint32             */
#define CPS_CHR_SENSOR_LOCATION_UUID16  0x2A5D /* read, uint8              */
#define CPS_CHR_CONTROL_POINT_UUID16    0x2A66 /* write + indicate         */

/*
 * Appearance 0x0484 = (Category 0x12 Cycling << 6) | 0x04 Power Sensor.
 * NimBLE has no constant for this one; it does define
 * BLE_SVC_GAP_APPEARANCE_CYC_SPEED_AND_CADENCE_SENSOR == 1157, which is
 * subcategory 5 in the same category -- 1156 and 1157 being adjacent
 * confirms the arithmetic.
 */
#define CPS_APPEARANCE_POWER_SENSOR     0x0484

/*
 * Sensor Location 0x00 = "Other".
 * Honest: rider power comes from the bike's motor controller, not from a
 * crank/pedal/spider strain gauge.  Claiming "Left Crank" would be a lie
 * about a value some head units display.  Fallback if the watch dislikes
 * it: 0x05 Left Crank.
 */
#define CPS_SENSOR_LOCATION_OTHER       0x00

/*
 * CP Feature bitfield.  Only bit 3 is set.
 *
 *   bit  3  Crank Revolution Data Supported   <- the whole point; this is
 *                                               what makes cadence legal
 *                                               inside CPS
 * Notably CLEAR:
 *   bit  2  Wheel Revolution Data  -- LDI gives speed, not wheel revs.
 *                                     Synthesising them would fabricate a
 *                                     distance channel that fights the
 *                                     watch's GPS.  Setting it would also
 *                                     make the Control Point mandatory and
 *                                     oblige us to implement Set
 *                                     Cumulative Value.
 *   bit  9  Offset Compensation    -- the declarative half of "there is
 *                                     nothing to calibrate".
 *   bit 11  Multiple Sensor Locations
 *   bit 12  Crank Length Adjustment -- so the watch should keep the crank
 *                                      length it prompts for locally.
 *   bits 20-21 Distributed System Support = 00 (Unspecified/legacy), the
 *                                      maximally compatible value.
 */
#define CPS_FEATURE_CRANK_REVOLUTION_DATA   (1u << 3)
#define CPS_FEATURE_VALUE                   (CPS_FEATURE_CRANK_REVOLUTION_DATA)


/* ---- Device Information Service (hand-declared, see cps_gatt.c) ------ */
#define DIS_SVC_UUID16                  0x180A
#define DIS_CHR_MANUFACTURER_UUID16     0x2A29
#define DIS_CHR_MODEL_NUMBER_UUID16     0x2A24
#define DIS_CHR_SERIAL_NUMBER_UUID16    0x2A25
#define DIS_CHR_FIRMWARE_REV_UUID16     0x2A26

#define DIS_MANUFACTURER_NAME           "DIY"
#define DIS_MODEL_NUMBER                "eBike-PM-C3"
/*
 * The "-sim" suffix is deliberate: it makes a synthetic-data workout
 * self-identifying in the exported file forever.  Drop the suffix only when
 * the firmware is genuinely being fed by the bike.
 */
#define DIS_FIRMWARE_REVISION           "0.1.0-sim"

/* ---- Advertised device name ----------------------------------------- */
/* 31-byte adv budget: 24 used by flags+UUID+appearance+txpower+name. */
#define CPS_DEVICE_NAME                 "eBikePM-01"

/* ---- CP Measurement flags (uint16) ---------------------------------- */
#define CPS_MEAS_FLAG_PEDAL_POWER_BALANCE   (1u << 0)
#define CPS_MEAS_FLAG_ACCUM_TORQUE          (1u << 2)
#define CPS_MEAS_FLAG_WHEEL_REV_DATA        (1u << 4)
#define CPS_MEAS_FLAG_CRANK_REV_DATA        (1u << 5)

/* Maximum size of a packed CP Measurement given the flags we ever set. */
#define CPS_MEAS_MAX_LEN                    8

/*
 * ATT error code "Client Characteristic Configuration Descriptor Improperly
 * Configured".  NimBLE's BLE_ATT_ERR_* list stops at 0x13, so this is
 * defined here.  (The blecsc example uses 0x81, which is in the
 * application-specific range and means nothing.)
 */
#define CPS_ATT_ERR_CCCD_IMPROPERLY_CONFIGURED  0xFD

/* ---- API ------------------------------------------------------------- */

/*
 * Register GAP (0x1800), GATT (0x1801), DIS (0x180A) and the Cycling Power
 * Service.  Call before the NimBLE host task starts.
 */
esp_err_t cps_gatt_init(void);

/*
 * Pack a CP Measurement.
 *
 * Field order follows ascending flag-bit order after Instantaneous Power,
 * and the offsets are computed FROM THE FLAGS rather than hardcoded -- so
 * if pedal power balance (bit 0) is ever added it correctly shifts the
 * crank data instead of silently corrupting it.
 *
 * Returns the number of bytes written (8 with our flags).
 */
size_t cps_gatt_pack_measurement(uint8_t *buf, size_t buflen, int16_t power_w,
                                 uint16_t cum_crank_rev,
                                 uint16_t last_crank_evt_1024);

/* Send one notification if the watch is subscribed.  Never asserts. */
void cps_gatt_notify(uint16_t conn_handle, int16_t power_w,
                     uint16_t cum_crank_rev, uint16_t last_crank_evt_1024);

/* Attribute handles, filled in at registration. */
extern uint16_t g_cps_measurement_handle;
extern uint16_t g_cps_control_point_handle;

#ifdef __cplusplus
}
#endif
