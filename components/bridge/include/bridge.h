/*
 * bridge -- orchestration.  The only module that knows about both BLE
 * roles.  Everything else is single-role and independently testable.
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bring up NVS-backed bonding, the NimBLE host, the GATT database and the
 * data source, then start the host task.  Returns once the host task is
 * running; advertising begins from the sync callback.
 */
esp_err_t bridge_start(void);

/* Drop every stored bond.  Used by the serial console and the BOOT button. */
void bridge_clear_bonds(void);

/* Terminate the watch link; advertising resumes from the disconnect event. */
void bridge_disconnect_watch(void);

#ifdef __cplusplus
}
#endif
