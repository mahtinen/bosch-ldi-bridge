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

/* ---- Cycling Speed and Cadence, the same cadence under a second name -- */

/*
 * WHY A SECOND SERVICE FOR DATA WE ALREADY SEND
 *
 * Cadence has always ridden inside the CP Measurement as crank revolution
 * data, and anything that speaks CPS gets it. But a watch does not choose
 * sensors by what they can do -- it types them once, at pairing, from what
 * they advertise, and then each sport mode uses the types it cares about.
 *
 * Measured on a Suunto Race S: with only 0x1818 advertised, the pod is typed
 * "Power". eMTB lists it as connected; normal Cycling connects to it, sub-
 * scribes, and then leaves it out of the sensor list entirely. Same sensor,
 * same link, different mode -- the watch is filtering on the stored type.
 *
 * Advertising 0x1816 as well lets it be typed as a bike pod too, which every
 * cycling mode should accept. No new data, no new arithmetic: the two values
 * below are the same pair crank_model_get() already hands to CPS.
 *
 * Note this only takes effect after the watch FORGETS and RE-PAIRS the pod.
 * The stored type is never revisited.
 */
#define CSC_SVC_UUID16                  0x1816 /* Cycling Speed and Cadence */
#define CSC_CHR_MEASUREMENT_UUID16      0x2A5B /* notify only, uint8 flags  */
#define CSC_CHR_FEATURE_UUID16          0x2A5C /* read, uint16              */

/*
 * CSC Feature bitfield, uint16. Only bit 1.
 *
 *   bit 1  Crank Revolution Data Supported
 *
 * Notably CLEAR:
 *   bit 0  Wheel Revolution Data -- LDI gives speed, not wheel revolutions,
 *                                   and synthesising them needs a wheel
 *                                   circumference we do not have. Setting it
 *                                   would also make the SC Control Point
 *                                   mandatory (Set Cumulative Value).
 *   bit 2  Multiple Sensor Locations -- one location, so Sensor Location is
 *                                   optional and the Control Point with it.
 */
#define CSC_FEATURE_VALUE               0x0002

/* CSC Measurement flags, uint8 (note: uint8, where CPS uses uint16). */
#define CSC_MEAS_FLAG_WHEEL_REV_DATA        (1u << 0)
#define CSC_MEAS_FLAG_CRANK_REV_DATA        (1u << 1)

/* flags(1) + cumulative crank revs(2) + last crank event time(2). */
#define CSC_MEAS_MAX_LEN                5

/*
 * Appearance 0x0483 = (Category 0x12 Cycling << 6) | 0x03 Cadence Sensor --
 * adjacent to the power sensor's 0x0484, which is the arithmetic check.
 */
#define CSC_APPEARANCE_CADENCE_SENSOR   0x0483

/*
 * The cadence identity advertises under its own name so the two are
 * distinguishable in the watch pairing list.  They are the same board.
 */
#define CSC_DEVICE_NAME                 "eBikeCAD-01"

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
 * Firmware revision is built at RUNTIME, not hardcoded.
 *
 * It used to read "0.1.0-sim", with a comment explaining that the suffix made
 * synthetic-data workouts self-identifying forever. By the time the bike side
 * worked, that string was marking every genuine ride as simulated -- the tag
 * asserting the opposite of the truth, which is worse than no tag.
 *
 * Now it is `git describe` plus the build timestamp, straight from the app
 * descriptor, so eBike Flow's accessory software-version field answers a
 * question that actually matters during development: is the bike talking to
 * the build I just flashed, or a stale one?
 */
#define DIS_FIRMWARE_REV_MAX            64

/* Also published as Software Revision (0x2A28): which of the two a given app
 * displays is not something the spec pins down, so publish both. */
#define DIS_CHR_SOFTWARE_REV_UUID16     0x2A28
/* ---- Advertised device name ----------------------------------------- */
/*
 * Back to the full name, in the scan response.
 *
 * It was briefly cut to four characters to mirror a working third-party
 * accessory byte-for-byte -- name in the advertising data, no scan response,
 * no 0x1818. That experiment did not make the eBike accept us, and discovery
 * got worse rather than better while it was in place, so it is reverted. The
 * 31-byte budget does not fit a useful name alongside flags, appearance, the
 * 128-bit solicitation and the service UUID list; the scan response is where
 * spec 2.1.5.3.2 explicitly allows it, and the watch reads it there.
 */
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

/* The accessory software version string published in DIS (0x2A26 / 0x2A28),
 * i.e. exactly what eBike Flow shows. */
const char *cps_gatt_firmware_revision(void);

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

/*
 * Pack a CSC Measurement, crank data only.
 *
 * Deliberately not derived from cps_gatt_pack_measurement(): the flags field
 * is uint8 here and uint16 there, and the two services order their optional
 * fields differently.  Sharing the packer would be the shortest route to
 * putting CPS bytes on a CSC characteristic.
 *
 * Returns the number of bytes written (5).
 */
size_t csc_gatt_pack_measurement(uint8_t *buf, size_t buflen,
                                 uint16_t cum_crank_rev,
                                 uint16_t last_crank_evt_1024);

/* Send one CSC notification.  Same never-assert contract as above. */
void csc_gatt_notify(uint16_t conn_handle, uint16_t cum_crank_rev,
                     uint16_t last_crank_evt_1024);

/* Attribute handles, filled in at registration. */
extern uint16_t g_cps_measurement_handle;
extern uint16_t g_cps_control_point_handle;
extern uint16_t g_csc_measurement_handle;

#ifdef __cplusplus
}
#endif
