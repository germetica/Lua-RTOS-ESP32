/*
 * Copyright (C) 2015 - 2018, IBEROXARXA SERVICIOS INTEGRALES, S.L.
 * Copyright (C) 2015 - 2018, Jaume Olivé Petrus (jolive@whitecatboard.org)
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *     * The WHITECAT logotype cannot be changed, you can remove it, but you
 *       cannot change it in any way. The WHITECAT logotype is:
 *
 *          /\       /\
 *         /  \_____/  \
 *        /_____________\
 *        W H I T E C A T
 *
 *     * Redistributions in binary form must retain all copyright notices printed
 *       to any local or remote output device. This include any reference to
 *       Lua RTOS, whitecatboard.org, Lua, and other copyright notices that may
 *       appear in the future.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Lua RTOS, BT driver
 *
 */

#include "sdkconfig.h"

#if CONFIG_BT_ENABLED

#ifndef BT_H_
#define BT_H_

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "bluetooth_hci.h"
#include "esp_gap_ble_api.h"

#include <stdint.h>

#include <sys/driver.h>

// Includes para BR/EDR (Bluetooth Classic)
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"

// BT modes
typedef enum {
	Idle = 0,
	BLE = 1,
	Classic = 2,
	Dual = 3
} bt_mode_t;

typedef enum {
	BTAdvUnknown = 0,
	BTAdvEddystoneUID = 1,
	BTAdvEddystoneURL = 2,
	BTAdvBrEdrInqRsp = 3 // Agregado: BR/EDR Inquiry Response
} bt_adv_frame_type_t;

typedef struct {
	int     rssi;
	uint8_t raw[31];
	uint8_t len;
	uint8_t flags;
	uint8_t bd_addr[ESP_BD_ADDR_LEN]; // Agregado: BD_ADDR del dispositivo emisor

	bt_adv_frame_type_t frame_type;
	union {
		struct {
			int8_t  tx_power;
			float   distance;
			uint8_t namespace[10];
			uint8_t instance[6];
		} eddystone_uid;

		struct {
			int8_t  tx_power;
			float   distance;
			uint8_t url[100];
		} eddystone_url;
		
		struct { // Agregado: Nombre del dispositivo BR/EDR
			char bdname[ESP_BT_GAP_MAX_BDNAME_LEN];
			uint8_t bdname_len;
		} bredr_inq_rsp;
	} data;
} bt_adv_frame_t;

typedef void (*bt_scan_callback_t)(int, bt_adv_frame_t *);

typedef uint8_t bt_adress_t[6];

// BT errors
#define BT_ERR_CANT_INIT			 	 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  0)
#define BT_ERR_INVALID_MODE   		 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  1)
#define BT_ERR_IS_NOT_SETUP   		 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  2)
#define BT_ERR_NOT_ENOUGH_MEMORY	 	 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  3)
#define BT_ERR_INVALID_ARGUMENT	 	 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  4)
#define BT_ERR_INVALID_BEACON	 	 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  5)
#define BT_ERR_CANT_START_SCAN	 	 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  6)
#define BT_ERR_CANT_STOP_SCAN	 	 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  7)
#define BT_ERR_CANT_START_ADV		 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  8)
#define BT_ERR_CANT_STOP_ADV			 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) |  9)
#define BT_ERR_INVALID_TX_POWER		 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) | 10)
#define BT_ERR_ADVDATA_TOO_LONG		 (DRIVER_EXCEPTION_BASE(BT_DRIVER_ID) | 11)

driver_error_t *bt_setup(bt_mode_t mode);
driver_error_t *bt_adv_start(bte_advertise_params_t params, uint8_t *adv_data, uint16_t adv_data_len);
driver_error_t *bt_adv_stop();
driver_error_t *bt_scan_start(bt_scan_callback_t cb, int cb_id);
driver_error_t *bt_scan_stop();
driver_error_t *bt_setup_dual(uint8_t bt_mode, uint8_t scan_bredr_mode, const char *dev_bredr_name); // Agregado: Setup para Modo dual
driver_error_t *bt_scan_set_le_duration(uint32_t ble_duration, uint16_t scan_interval, uint16_t scan_window); // Agregado: Setear duración de LE
driver_error_t *bt_scan_set_bredr_duration(uint8_t bredr_inq_len); // Agregado: Setear duración de BR/EDR Inquiry (*1.28 s)
driver_error_t *bt_scan_set_min_rssi(int min_rssi); // Agregado: Setear el valor mínimo de RSSI permitido para Scan LE y Discovery BR/EDR (0 es sin mínimo)
driver_error_t *bt_scan_start_le(bt_scan_callback_t cb, int cb_id); // Agregado: Scan LE
driver_error_t *bt_scan_start_bredr(bt_scan_callback_t cb, int cb_id); // Agregado: Scan BR/EDR
driver_error_t *bt_scan_stop_bredr(); // Agregado: Para detener un scan BR/EDR
driver_error_t *bt_is_le_scanning(bool *isLeScanning); // Agregado: Indica si se está ejecutando un Scan LE
driver_error_t *bt_is_bredr_scanning(bool *isBrEdrScanning); // Agregado: Indica si se está ejecutando un Scan BR/EDR
driver_error_t *bt_get_bdaddr(uint8_t *bd_addr); // Agregado: Obtener BD_ADDR de este dispositivo
driver_error_t *bt_free_mem(); // Agregado: Para liberar la memoria del componente Bluetooth, en caso de que no se quiera usar

extern const int bt_errors;
extern const int bt_error_map;

#include "bluetooth_eddystone.h"

#endif /* BT_H_ */

#endif
