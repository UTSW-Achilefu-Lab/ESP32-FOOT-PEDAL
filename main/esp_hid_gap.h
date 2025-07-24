/* esp_hid_gap.h
 *
 * modified from esp-idf bluetooth device example, unused functions removed
 * Copyright (C) 2025 Ian Zurutuza
 *
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef _ESP_HID_GAP_H_
#define _ESP_HID_GAP_H_

#define HIDD_BLE_MODE 0x01

#define HID_DEV_MODE HIDD_BLE_MODE

#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"

#include "esp_hid_common.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t esp_hid_gap_init(uint8_t mode);

esp_err_t esp_hid_ble_gap_adv_init(uint16_t appearance, const char *device_name);
esp_err_t esp_hid_ble_gap_adv_start(void);

#ifdef __cplusplus
}
#endif

#endif /* _ESP_HIDH_GAP_H_ */
