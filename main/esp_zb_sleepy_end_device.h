/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee Sleepy end device Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#ifndef ESP_ZB_SLEEPY_END_DEVICE_H
#define ESP_ZB_SLEEPY_END_DEVICE_H

#include "esp_zigbee_core.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE false                                  /* enable the install code policy for security */
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN                   /* Child aging timeout (ms), this value means that end device doesn't need to perform check-in to stack aging mechanism */
#define ED_KEEP_ALIVE 3000                                               /* End device keep alive timeout in ms. 0 - deactivated, when less than poll timeout, then poll timeout is used for short sleep */
#define HA_ESP_LIGHT_ENDPOINT 10                                         /* esp light bulb device endpoint, used to process light controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/* Power source defines */
#define ZB_POWER_SOURCE_BATTERY 0x03 /* ZCL Power Source value for Battery */

#define NVS_NAMESPACE "servo"
#define NVS_KEY "position"
#define NVS_KEY_CALIBRATION "calibration"

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09" \
                              "ESPRESSIF"             /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07" CONFIG_IDF_TARGET /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                               \
    {                                                     \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,             \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg = {                              \
            .ed_timeout = ED_AGING_TIMEOUT,               \
            .keep_alive = ED_KEEP_ALIVE,                  \
        },                                                \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()       \
    {                                       \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                          \
    {                                                         \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }

/* Константы для времени сна */
#define DEFAULT_SLEEP_TIME_SEC 60

/* Фукнции обработки сигналов Zigbee */
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);

/* Функция запуска калибровки сервопривода */
esp_err_t run_servo_calibration(void);

/* Define a common command: set the siren volume. */
#define ESP_ZB_ZCL_CMD_WINDOW_COVERING_GOTO_LIFT_VALUE 0x04
#define ESP_ZB_ZCL_CMD_WINDOW_COVERING_GOTO_LIFT_PERCENTAGE 0x05
#define ESP_ZB_ZCL_CMD_WINDOW_COVERING_GOTO_TILT_VALUE 0x08
#define ESP_ZB_ZCL_CMD_WINDOW_COVERING_GOTO_TILT_PERCENTAGE 0x09

/* --- ВОТ ЭТО ИЗМЕНЕНО --- */
#define ESP_ZB_SLEEP_TIMEOUT_MS (1000) /* Sleep timeout in ms */

/* Attribute ID used by application */
#define MOTOR_ATTRIBUTE_ID (0x0002) /* actual value, 1/10 */

#define CONFIG_TARGET "Sleepy End Device" /* For compile time check */

#define NO_SLEEP_DISABLE_TIMEOUT 60000 /* 60 seconds */

/* Source endpoint */
#define HA_ENDPOINT 10

#define SLEEPY_AGING_SLEEP_MS 60000 /* 60 seconds */

#endif // ESP_ZB_SLEEPY_END_DEVICE_H
