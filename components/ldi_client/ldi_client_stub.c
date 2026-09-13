/*
 * No-op LDI client, linked when CONFIG_BRIDGE_ENABLE_LDI=n.
 *
 * Exists so the rest of the firmware links and runs with the bike half
 * compiled out entirely -- the watch-first build order is then a config flag
 * rather than a maze of #ifdefs in the supervisor.
 */
#include "ldi_client.h"

#include <stdio.h>

esp_err_t ldi_client_scan(int32_t duration_ms) { (void)duration_ms; return ESP_ERR_NOT_SUPPORTED; }
void      ldi_client_scan_stop(void) {}

void      ldi_client_offer_inbound(uint16_t h) { (void)h; }
bool      ldi_client_owns(uint16_t h) { (void)h; return false; }
int       ldi_client_gap_event(struct ble_gap_event *ev) { (void)ev; return 0; }

void      ldi_client_disconnect(void) {}
bool      ldi_client_is_connected(void) { return false; }
void      ldi_client_supervise(void) {}
uint32_t  ldi_client_notify_total(void) { return 0; }

void      ldi_client_set_dump(bool on) { (void)on; }
bool      ldi_client_get_dump(void) { return false; }
void      ldi_client_print_gatt(void) { printf("LDI accessory not compiled in\n"); }
void      ldi_client_print_status(void) { printf("LDI accessory not compiled in (CONFIG_BRIDGE_ENABLE_LDI=n)\n"); }

void ldi_client_on_sync(uint8_t own_addr_type) { (void)own_addr_type; }
