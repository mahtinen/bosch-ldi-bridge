/*
 * Bosch eBike LDI -> BLE Cycling Power bridge.
 *
 * Brings up two BLE roles on one radio: a central subscribing to the
 * bike's LiveData Interface, and a peripheral advertising a standard
 * Cycling Power Service to the watch.
 *
 * See README.md for an overview and docs/LDI-PROTOCOL.md for what the
 * bike actually sends.
 */
#include "bridge.h"
#include "status_led.h"

#include "esp_log.h"

static const char *TAG = "app";

void app_main(void)
{
    ESP_LOGI(TAG, "-----------------------------------------");
    ESP_LOGI(TAG, " eBike -> BLE power meter bridge");
    ESP_LOGI(TAG, " bosch-ldi-bridge");
    ESP_LOGI(TAG, "-----------------------------------------");

    status_led_init();
    ESP_ERROR_CHECK(bridge_start());
}
