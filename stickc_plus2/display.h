#pragma once

#include "esp_err.h"

typedef enum {
    DISPLAY_STATUS_NOT_CONNECTED = 0,
    DISPLAY_STATUS_CONNECTED,
    DISPLAY_STATUS_RECORDING,
} display_status_t;

esp_err_t display_init(void);
void display_set_status(display_status_t status);
void display_sleep(void);

/* Blank the panel and park its pads so nothing floats through deep sleep. */
void display_prepare_deep_sleep(void);
