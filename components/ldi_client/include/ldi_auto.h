/*
 * ldi_auto -- internal interface between the LDI client and its autonomous
 * hunt state machine.  See ldi_auto.c for the rationale.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "host/ble_gap.h"

#ifdef __cplusplus
extern "C" {
#endif

void ldi_auto_init(void);
void ldi_auto_print_status(void);
void ldi_auto_clear_blacklist(void);
void ldi_auto_note_session(void);
bool ldi_auto_is_blacklisted(const ble_addr_t *a);

/* Provided by ldi_client.c for the state machine to use. */
uint32_t ldi_client_notify_total(void);
bool     ldi_client_peer_addr(ble_addr_t *out);
bool     ldi_client_best_candidate(ble_addr_t *out);

#ifdef __cplusplus
}
#endif
