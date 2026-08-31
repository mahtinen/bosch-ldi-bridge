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
esp_err_t ldi_client_connect(const char *a, uint8_t t) { (void)a; (void)t; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t ldi_client_connect_by_name(const char *s) { (void)s; return ESP_ERR_NOT_SUPPORTED; }
void      ldi_client_disconnect(void) {}
bool      ldi_client_is_connected(void) { return false; }
void      ldi_client_set_dump(bool on) { (void)on; }
bool      ldi_client_get_dump(void) { return false; }
void      ldi_client_print_gatt(void) { printf("LDI central not compiled in\n"); }
void      ldi_client_print_status(void) { printf("LDI central not compiled in (CONFIG_BRIDGE_ENABLE_LDI=n)\n"); }

void ldi_client_on_sync(uint8_t own_addr_type) { (void)own_addr_type; }

void      ldi_client_set_auto(bool on) { (void)on; }
bool      ldi_client_get_auto(void) { return false; }
esp_err_t ldi_client_set_target(const char *a, uint8_t t) { (void)a; (void)t; return ESP_ERR_NOT_SUPPORTED; }
bool      ldi_client_get_target(char *out, size_t n) { (void)out; (void)n; return false; }
