/*
 * cps_ctrl_point -- Cycling Power Control Point (0x2A66) handler.
 *
 * We support NO op codes.  Everything gets a well-formed
 * "Op Code Not Supported" indication.
 *
 * The reason to implement this at all, rather than omitting the
 * characteristic (it is optional for us -- Crank Revolution Data is not in
 * the CPS C.1 list that makes it mandatory): when the watch's "Calibrate
 * power POD" flow writes Start Offset Compensation, it gets a clean
 * immediate failure instead of an ATT "attribute not found" or a spinner
 * that never resolves.
 *
 * Response format, sent as an INDICATION on 0x2A66:
 *
 *   offset 0 : 0x20            Response Code    (CPS; CSCS uses 0x10!)
 *   offset 1 : <request opcode>  echoed verbatim -- the watch matches its
 *                                request to our response by this byte
 *   offset 2 : <response value>  1 Success, 2 Op Code Not Supported,
 *                                3 Invalid Parameter, 4 Operation Failed
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CPS_CP_RESPONSE_CODE            0x20 /* NOT 0x10 -- that is CSCS */

#define CPS_CP_RSP_SUCCESS              0x01
#define CPS_CP_RSP_OP_NOT_SUPPORTED     0x02
#define CPS_CP_RSP_INVALID_PARAMETER    0x03
#define CPS_CP_RSP_OPERATION_FAILED     0x04

/* Op codes we may see written (all rejected). */
#define CPS_CP_OP_SET_CUMULATIVE_VALUE  0x01
#define CPS_CP_OP_REQ_SENSOR_LOCATIONS  0x03
#define CPS_CP_OP_SET_CRANK_LENGTH      0x04
#define CPS_CP_OP_REQ_CRANK_LENGTH      0x05
#define CPS_CP_OP_START_OFFSET_COMP     0x0C /* "Calibrate power POD"     */
#define CPS_CP_OP_START_ENH_OFFSET_COMP 0x10

void cps_ctrl_point_init(void);

/* Called from the GAP subscribe event when the 0x2A66 CCCD changes. */
void cps_ctrl_point_set_indicate(uint16_t conn_handle, bool enabled);

/* Called on disconnect to drop any in-flight procedure state. */
void cps_ctrl_point_on_disconnect(uint16_t conn_handle);

/*
 * The 0x2A66 access callback body.  Returns an ATT error code, or 0 on
 * success -- in which case the indication has been QUEUED, not sent, so the
 * ATT Write Response goes out first as the spec requires.
 */
struct ble_gatt_access_ctxt;
int cps_ctrl_point_handle_write(uint16_t conn_handle,
                                struct ble_gatt_access_ctxt *ctxt);

#ifdef __cplusplus
}
#endif
