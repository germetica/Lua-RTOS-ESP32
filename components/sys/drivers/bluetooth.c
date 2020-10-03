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

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "bluetooth.h"
#include "esp_bt.h"

#include <stdint.h>
#include <string.h>

#include <sys/delay.h>
#include <sys/driver.h>
#include <sys/syslog.h>

DRIVER_REGISTER_BEGIN(BT,bt,0,NULL,NULL);
	DRIVER_REGISTER_ERROR(BT, bt, CannotSetup, "can't setup", BT_ERR_CANT_INIT);
	DRIVER_REGISTER_ERROR(BT, bt, InvalidMode, "invalid mode", BT_ERR_INVALID_MODE);
	DRIVER_REGISTER_ERROR(BT, bt, NotSetup, "is not setup", BT_ERR_IS_NOT_SETUP);
	DRIVER_REGISTER_ERROR(BT, bt, NotEnoughtMemory, "not enough memory", BT_ERR_NOT_ENOUGH_MEMORY);
	DRIVER_REGISTER_ERROR(BT, bt, InvalidArgument, "invalid argument", BT_ERR_INVALID_ARGUMENT);
	DRIVER_REGISTER_ERROR(BT, bt, InvalidBeacon, "invalid beacon", BT_ERR_INVALID_BEACON);
	DRIVER_REGISTER_ERROR(BT, bt, CannotStartScan, "can't start scanning", BT_ERR_CANT_START_SCAN);
	DRIVER_REGISTER_ERROR(BT, bt, CannotStopScan, "can't stop scanning", BT_ERR_CANT_STOP_SCAN);
	DRIVER_REGISTER_ERROR(BT, bt, CannotStartAdv, "can't start advertising", BT_ERR_CANT_START_ADV);
	DRIVER_REGISTER_ERROR(BT, bt, CannotStopAdv, "can't start advertising", BT_ERR_CANT_STOP_ADV);
	DRIVER_REGISTER_ERROR(BT, bt, InvalidTxPower, "invalid tx power", BT_ERR_INVALID_TX_POWER);
	DRIVER_REGISTER_ERROR(BT, bt, AdvTooLong, "advertising data must be less then 31 bytes", BT_ERR_ADVDATA_TOO_LONG);
DRIVER_REGISTER_END(BT,bt,0,NULL,NULL);

#define evBT_SCAN_START_COMPLETE ( 1 << 0 )
#define evBT_SCAN_START_ERROR    ( 1 << 1 )
#define evBT_SCAN_STOP_COMPLETE  ( 1 << 2 )
#define evBT_SCAN_STOP_ERROR     ( 1 << 3 )
#define evBT_ADV_START_COMPLETE  ( 1 << 4 )
#define evBT_ADV_START_ERROR     ( 1 << 5 )
#define evBT_ADV_STOP_COMPLETE   ( 1 << 6 )
#define evBT_ADV_STOP_ERROR      ( 1 << 7 )

#define GAP_CB_CHECK(s, ok_ev, error_ev) \
if(s != ESP_BT_STATUS_SUCCESS) { \
	xEventGroupSetBits(bt_event, error_ev); \
} else { \
	xEventGroupSetBits(bt_event, ok_ev); \
	break; \
}

// Is BT setup?
static uint8_t setup = 0;
static uint8_t setup_dual = 0; // Agregado: Flag de si se configuró el modo Dual
static esp_bt_mode_t bt_mode_set = ESP_BT_MODE_IDLE; // Agregado: Modo BT configurado

// Event group to sync some driver functions with the event handler
EventGroupHandle_t bt_event;

// Queue and task to get data generated in the event handler
static xQueueHandle queue = NULL;
static TaskHandle_t task = NULL;

// Tiempo a esperar cuando se envía a queue y ésta está llena.
// Si se define en 0 (como estaba originalmente para BLE), se perderán paquetes si la función de callback LuaRTOS demora mucho en
// procesar el frame y provoca que la cola se llene.
// Si QUEUE_TICKS_TO_WAIT != 0, la función de callback LuaRTOS no debería demorar para evitar el llenado del buffer, además de la
// pérdida de paquetes.
// Si se define en portMAX_DELAY, al llenarse la cola, al intentar enviar se bloqueará hasta que haya lugar en la cola. Esto podría
// bloquear indefinidamente el hilo que envía los paquetes a la función de callback del scan BT, provocando el llenado del buffer
// y haciendo que se pierdan los demás paquetes.
// Tener en cuenta que se usa la misma cola para BLE y BR/EDR.
#define QUEUE_TICKS_TO_WAIT		(100/portTICK_PERIOD_MS)/*portMAX_DELAY*/

bt_scan_callback_t callback;
static int callback_id;

// Para mostrar o no mensajes de debug
#define MENSAJES_DEBUG			0
#define BREDR_MIN_VALID_RSSI	-128		/* Según la API (esp_gap_bt_api.h), el intervalo válido del RSSI para BR/EDR es [-128, 127] */
#define BREDR_MAX_VALID_RSSI	 127
#define BREDR_GET_BDNAME		0			/* Poner en 1 si se quiere obtener el nombre del dispositivo */
static uint32_t ble_scan_duration = 0;		// Agregado: Duración para escaneo BLE cuando se configuró con bt_setup_dual
static uint8_t bredr_scan_inq_len = 1;		// Agregado: Duración del Inquiry BR/EDR. La unidad son 1.28 s
static bool ble_scan_running = false;		// Agregado: Indica si se está ejecutando un Scan LE
static bool bredr_scan_running = false;		// Agregado: Indica si se está ejecutando un Scan BR/EDR
static bool ble_scan_params_set = false;	// Agregado: Para saber si los parámetros de scan ya están seteados
static bool ble_process_eddystone = true;	// Agregado: Con bt_setup_dual, no se va a a procesar Eddystone
static int scan_min_rssi = 0;				// Agregado: Setear el valor mínimo de RSSI permitido para Scan LE y Discovery BR/EDR (0 es sin mínimo)

/*
 * Helper functions
 */
static void bt_task(void *arg) {
	bt_adv_frame_t data;

	memset(&data,0,sizeof(bt_adv_frame_t));

    for(;;) {
        xQueueReceive(queue, &data, portMAX_DELAY);
        callback(callback_id, &data);
    }
}


#if MENSAJES_DEBUG
// INI Agregado - Funciones Auxiliares
static void printMac(uint8_t *mac, char *tadd1, char *tadd2, char *file, int line) {
    printf("[%s:%d] %s%02x:%02x:%02x:%02x:%02x:%02x%s\n", file, line, tadd1, mac[0],mac[1],mac[2],mac[3],mac[4],mac[5], tadd2);
}
static void printStrLen(char *str, int len, char *taddNoNull, char *taddNull, char *file, int line) {
    if (str) {
        printf("[%s:%d] %s", file, line, taddNoNull);
        for (int i = 0; i < len; i++)
            printf("%c", str[i]);
        printf(" (%d)\n", len);
    }
    else
        printf("[%s:%d] %s (%d)\n", file, line, taddNull, len);
}
// FIN Agregado - Funciones Auxiliares
#endif


static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
	switch (event) {
		case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
			GAP_CB_CHECK(param->adv_start_cmpl.status, evBT_ADV_START_COMPLETE, evBT_ADV_START_ERROR);
			break;
		}

		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: {
			GAP_CB_CHECK(param->adv_stop_cmpl.status, evBT_ADV_STOP_COMPLETE, evBT_ADV_STOP_ERROR);
			break;
		}

		case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
			uint32_t duration = !setup_dual ? 0 : ble_scan_duration;
			ble_scan_params_set = true;
			esp_ble_gap_start_scanning(duration);
#if MENSAJES_DEBUG
			printf("[%s:%d (%s)] [BLE_SCAN_PARM_SET_COMPLETE]\n", __FILE__, __LINE__, __func__);
#endif
			break;
		}

		case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
#if MENSAJES_DEBUG
			printf("[%s:%d (%s)] [BLE_SCAN_START_COMPLETE]\n", __FILE__, __LINE__, __func__);
#endif
			if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
				ble_scan_running = true;
			GAP_CB_CHECK(param->scan_start_cmpl.status, evBT_SCAN_START_COMPLETE, evBT_SCAN_START_ERROR);
			break;
		}

	case ESP_GAP_BLE_SCAN_RESULT_EVT: {
		esp_ble_gap_cb_param_t* scan_result = param;

		switch (scan_result->scan_rst.search_evt) {
			case ESP_GAP_SEARCH_INQ_RES_EVT: {
				if (scan_min_rssi == 0 || scan_result->scan_rst.rssi >= scan_min_rssi) {
					bt_adv_frame_t frame;

					// Get RSSI
					frame.rssi = scan_result->scan_rst.rssi;

					// Get raw data
					frame.len = scan_result->scan_rst.adv_data_len;
					memcpy(frame.raw, scan_result->scan_rst.ble_adv, frame.len);
					// INI Agregado: se asigna nuevo campo con BD_ADDR del dispositivo visto
					memcpy(frame.bd_addr, scan_result->scan_rst.bda, 6);
#if MENSAJES_DEBUG
						uint8_t *bda = frame.bd_addr; // BD_ADDR del emisor
						printf("[%s:%d (%s)] bd_addr: %02x:%02x:%02x:%02x:%02x:%02x | RSSI: %d\n", __FILE__, __LINE__, __func__,
								bda[0],bda[1],bda[2],bda[3],bda[4],bda[5], scan_result->scan_rst.rssi);
#endif
					// FIN Agregado

					if (ble_process_eddystone) {
						bt_eddystone_decode(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, &frame);
					} else {
						frame.frame_type = BTAdvUnknown;
					}
					//xQueueSend(queue, &frame, 0);
					if (xQueueSend(queue, &frame, QUEUE_TICKS_TO_WAIT) != pdPASS) { // Originalmente, era 0. Sin espera si la cola está llena
						printf("(%s) LE packet lost\n", __func__);
					}
				}

				break;
			}
			case ESP_GAP_SEARCH_INQ_CMPL_EVT: {
#if MENSAJES_DEBUG
				printf("[%s:%d (%s)] [ESP_GAP_SEARCH_INQ_CMPL_EVT] (Tiempo de Scan expiró)\n", __FILE__, __LINE__, __func__);
#endif
				ble_scan_running = false;
				// ESP_BT_STATUS_SUCCESS: parte del enumerado esp_bt_status_t (esp_bt_defs.h)
				//GAP_CB_CHECK(ESP_BT_STATUS_SUCCESS, evBT_SCAN_STOP_COMPLETE, evBT_SCAN_STOP_ERROR);
				break;
			}

			default:
				break;
		}

		break;
	}

	case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
		//if (param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS)
			ble_scan_running = false;
#if MENSAJES_DEBUG
		printf("[%s:%d (%s)] [BLE_SCAN_STOP_COMPLETE] (detenido con esp_ble_gap_stop_scanning())\n", __FILE__, __LINE__, __func__);
#endif
		GAP_CB_CHECK(param->scan_stop_cmpl.status, evBT_SCAN_STOP_COMPLETE, evBT_SCAN_STOP_ERROR);
		break;
	}

	default:
		break;
	}
}

/*
 * Operation functions
 */

driver_error_t *bt_setup(bt_mode_t mode) {
	// Sanity checks
	if (setup) {
		return NULL;
	}

	if (mode > Dual) {
		return driver_error(BT_DRIVER, BT_ERR_INVALID_MODE, NULL);
	}

	// Create an event group sync some driver functions with the event handler
	bt_event = xEventGroupCreate();

	queue = xQueueCreate(10, sizeof(bt_adv_frame_t));
	if (!queue) {
		return driver_error(BT_DRIVER, BT_ERR_NOT_ENOUGH_MEMORY, NULL);
	}

	BaseType_t xReturn = xTaskCreatePinnedToCore(bt_task, "bt", CONFIG_LUA_RTOS_LUA_THREAD_STACK_SIZE, NULL, CONFIG_LUA_RTOS_LUA_THREAD_PRIORITY, &task, xPortGetCoreID());
	if (xReturn != pdPASS) {
		return driver_error(BT_DRIVER, BT_ERR_NOT_ENOUGH_MEMORY, NULL);
	}

	// Initialize BT controller
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);

	// Enable BT controller
	if (esp_bt_controller_enable(mode) != ESP_OK) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_INIT, NULL);
	}

    esp_bluedroid_init();
    esp_bluedroid_enable();
	esp_ble_gap_register_callback(&gap_cb);

	setup = 1;

	return NULL;
}


// INI Agregado - setup Dual Mode (BR/EDR y LE)
#if BREDR_GET_BDNAME
static void getNameFromEir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
	if (!eir || !bdname)
		return;
	// Se obtiene el nombre, si está presente en el EIR (Extended Inquiry Response)
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;
	
    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname)
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN)
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        memcpy(bdname, rmt_bdname, rmt_bdname_len);
        if (bdname_len)
            *bdname_len = rmt_bdname_len;
    }
}
#endif


static void getDeviceInfo(esp_bt_gap_cb_param_t *param, uint8_t *bdaddr, int *rssi, char *bdname, uint8_t *bdname_len) {
#if BREDR_GET_BDNAME
	uint8_t eir_len = 0;
    uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
#endif
	
	*rssi = BREDR_MIN_VALID_RSSI - 1;
	*bdname_len = 0;
	memcpy(bdaddr, param->disc_res.bda, ESP_BD_ADDR_LEN);
	
	esp_bt_gap_dev_prop_t *p;
	for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
         case ESP_BT_GAP_DEV_PROP_RSSI:
            *rssi = *(int8_t *)(p->val);
            break;
#if BREDR_GET_BDNAME	/* Si no interesa el nombre del dispositivo, esta parte se excluye */
		 case ESP_BT_GAP_DEV_PROP_BDNAME: {
			if (bdname) { // Si bdname es NULL, no se quiere el nombre
            	*bdname_len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN : (uint8_t)p->len;
            	memcpy(bdname, (uint8_t *)(p->val), *bdname_len);
			}
            break;
         }
#endif
#if BREDR_GET_BDNAME /* Por el momento, el EIR se usa solo para obtener el nombre */
         case ESP_BT_GAP_DEV_PROP_EIR: {
			if (bdname) { // Si bdname es NULL, no se quiere el nombre
            	memcpy(eir, (uint8_t *)(p->val), p->len);
            	eir_len = p->len;
			}
            break;
         }
#endif
         default:
            break;
        }//switch
    }//for
#if BREDR_GET_BDNAME
	if (bdname && eir_len > 0 && *bdname_len == 0) // Si bdname es NULL, no se quiere el nombre
        getNameFromEir(eir, (uint8_t*)bdname, bdname_len);
#endif
}


void bt_bredr_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	switch (event) {
     case ESP_BT_GAP_DISC_RES_EVT: {
		bt_adv_frame_t frame;
		frame.frame_type = BTAdvBrEdrInqRsp; // BR/EDR Inquiry Response
		
		// Datos BLE
		frame.len = 0; frame.flags = 0;
		
		getDeviceInfo(param, frame.bd_addr, &frame.rssi, frame.data.bredr_inq_rsp.bdname, &frame.data.bredr_inq_rsp.bdname_len);
		
#if MENSAJES_DEBUG
		printMac(frame.bd_addr, "[ESP_BT_GAP_DISC_RES_EVT] Dispositivo (BR/EDR): ", "", __FILE__, __LINE__);
        printStrLen(frame.data.bredr_inq_rsp.bdname, frame.data.bredr_inq_rsp.bdname_len, "    Nombre dispositivo: ", "    Dispositivo sin nombre", __FILE__, __LINE__);
		printf("[%s:%d (%s)]     RSSI: %d\n", __FILE__, __LINE__, __func__, frame.rssi);
#endif
		if (BREDR_MIN_VALID_RSSI <= frame.rssi && frame.rssi <= BREDR_MAX_VALID_RSSI) {
			if (scan_min_rssi == 0 || frame.rssi >= scan_min_rssi) {
				if (xQueueSend(queue, &frame, QUEUE_TICKS_TO_WAIT) != pdPASS) {
					printf("(%s) BR/EDR packet lost\n", __func__);
				}
			}
		}
        break;
     }
     case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
		// Discovery started or halted
		if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
			bredr_scan_running = true;
#if MENSAJES_DEBUG
            printf("[%s:%d (%s)] Device discovery started.\n", __FILE__, __LINE__, __func__);
#endif
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
			bredr_scan_running = false;
#if MENSAJES_DEBUG
			printf("[%s:%d (%s)] Device discovery stopped.\n", __FILE__, __LINE__, __func__);
#endif
        }
        break;
     }
     default: {
        //printf("[%s:%d (%s)] event: %d\n", __FILE__, __LINE__, __func__, event);
        break;
	 }
    }
    return;
}


/**
 * @brief       Similar a bt_setup, pero permite configurar solo LE, solo BR/EDR o ambos. Libera la memoria del modo que no se va a usar. En caso
 *              de que no se vaya a usar el componente Bluetooth, se recomienda ejecutar bt_freemem().
 * @param[in]   bt_mode: 0: BLE; 1: BR/EDR; 2: Dual Mode
 * @param[in]   scan_bredr_mode: Modo escaneable de BR/EDR: 0: ESP_BT_SCAN_MODE_NONE; 1: ESP_BT_SCAN_MODE_CONNECTABLE; 2: ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE
 * @param[in]   dev_bredr_name: Nombre del dispositivo para el modo BR/EDR (o dual) en caso de que quiera ser descubierto
 */
driver_error_t *bt_setup_dual(uint8_t bt_mode, uint8_t scan_bredr_mode, const char *dev_bredr_name) {
	// Sanity checks
	if (setup) {
		return NULL;
	}

	if (bt_mode == 0) { // BLE
		bt_mode_set = ESP_BT_MODE_BLE;
		if (esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) == ESP_OK) printf("ESP_BT_MODE_CLASSIC_BT memory released\n");
		else printf("(%s) Error trying to release ESP_BT_MODE_CLASSIC_BT memory\n", __func__);
	} else if (bt_mode == 1) { // BR/EDR
		bt_mode_set = ESP_BT_MODE_CLASSIC_BT;
		if (esp_bt_controller_mem_release(ESP_BT_MODE_BLE) == ESP_OK) printf("ESP_BT_MODE_BLE memory released\n");
		else printf("(%s)] Error trying to release ESP_BT_MODE_BLE memory\n", __func__);
	} else if (bt_mode == 2) { // Dual
		bt_mode_set = ESP_BT_MODE_BTDM;
	} else { // Otro
		return driver_error(BT_DRIVER, BT_ERR_INVALID_ARGUMENT, NULL);
	}
	
	// Create an event group sync some driver functions with the event handler
	bt_event = xEventGroupCreate();
	
	queue = xQueueCreate(10, sizeof(bt_adv_frame_t)); // Máximo, 10 ítems del tamaño de bt_adv_frame_t
	if (!queue)
		return driver_error(BT_DRIVER, BT_ERR_NOT_ENOUGH_MEMORY, NULL);
	
	BaseType_t xReturn = xTaskCreatePinnedToCore(bt_task, "bt", CONFIG_LUA_RTOS_LUA_THREAD_STACK_SIZE, NULL, CONFIG_LUA_RTOS_LUA_THREAD_PRIORITY, &task, xPortGetCoreID());
	if (xReturn != pdPASS) {
		return driver_error(BT_DRIVER, BT_ERR_NOT_ENOUGH_MEMORY, NULL);
	}
	
	// Initialize BT controller
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);
	
	// Enable BT controller - Dual Mode
	if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_INIT, NULL);
	}
	
    esp_bluedroid_init();
    esp_bluedroid_enable();

	if (bt_mode_set == ESP_BT_MODE_BLE || bt_mode_set == ESP_BT_MODE_BTDM) { // LE o Dual
		esp_ble_gap_register_callback(&gap_cb);
	}
	
	// Configuración BR/EDR:
	if (bt_mode_set == ESP_BT_MODE_CLASSIC_BT || bt_mode_set == ESP_BT_MODE_BTDM) { // BR/EDR o Dual
		esp_bt_scan_mode_t esp_bt_scan_mode;
		if (scan_bredr_mode == 0) {
			esp_bt_scan_mode = ESP_BT_SCAN_MODE_NONE; /*!< Neither discoverable nor connectable */
		} else if (scan_bredr_mode == 1) {
			esp_bt_scan_mode = ESP_BT_SCAN_MODE_CONNECTABLE; /*!< Connectable but not discoverable */
		} else if (scan_bredr_mode == 2) {
			esp_bt_scan_mode = ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE; /*!< both discoverable and connectable */
			if (dev_bredr_name && strlen(dev_bredr_name) > 0) {
				esp_bt_dev_set_device_name(dev_bredr_name);
			}
		} else
			return driver_error(BT_DRIVER, BT_ERR_INVALID_ARGUMENT, NULL);
		
		/* set discoverable and connectable mode, wait to be connected */
		esp_bt_gap_set_scan_mode(esp_bt_scan_mode);
		/* register GAP callback function */
		esp_bt_gap_register_callback(bt_bredr_cb);
	}
	
	ble_process_eddystone = false;
	setup = 1;
	setup_dual = 1;
	
	return NULL;
}
// FIN Agregado - setup Dual Mode (BR/EDR y LE)


driver_error_t *bt_reset() {
	driver_error_t *error;

	// Sanity checks
	if (!setup) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}

	if ((error = HCI_Reset())) {
		return error;
	}

	return NULL;
}

driver_error_t *bt_adv_start(bte_advertise_params_t adv_params, uint8_t *adv_data, uint16_t adv_data_len) {
	// Sanity checks
	if (!setup) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}

	// Set advertising data
	esp_ble_gap_config_adv_data_raw(adv_data, adv_data_len);

	// Start advertising
	esp_ble_adv_params_t params;

	params.adv_int_min = adv_params.interval_min;
	params.adv_int_max = adv_params.interval_max;
	params.adv_type = adv_params.type;
	params.own_addr_type = adv_params.own_address_type;
	memcpy(params.peer_addr,adv_params.peer_address, sizeof(params.peer_addr));
	params.peer_addr_type = adv_params.peer_address_type;
	params.channel_map = adv_params.chann_map;
	params.adv_filter_policy = adv_params.filter_policy;

	esp_ble_gap_start_advertising(&params);

	// Wait for advertising start completion
	EventBits_t uxBits = xEventGroupWaitBits(bt_event, evBT_ADV_START_COMPLETE | evBT_ADV_START_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);
	if (uxBits & (evBT_ADV_START_COMPLETE)) {
		return NULL;
	} else if (uxBits & (evBT_ADV_START_ERROR)) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_START_ADV, NULL);
	}

	return NULL;
}

driver_error_t *bt_adv_stop() {
	// Sanity checks
	if (!setup) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}

	// Stop advertising
	esp_ble_gap_stop_advertising();

	// Wait for advertising stop completion
	EventBits_t uxBits = xEventGroupWaitBits(bt_event, evBT_ADV_STOP_COMPLETE | evBT_ADV_STOP_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);
	if (uxBits & (evBT_ADV_STOP_COMPLETE)) {
		return NULL;
	} else if (uxBits & (evBT_ADV_STOP_ERROR)) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_STOP_ADV, NULL);
	}

	return NULL;
}

driver_error_t *bt_scan_start(bt_scan_callback_t cb, int cb_id) {
	// Sanity checks
	if (!setup) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}

	// Store callback data
	callback = cb;
	callback_id = cb_id;

	// Set scan parameters
	esp_ble_scan_params_t *scan_params = calloc(1, sizeof(esp_ble_scan_params_t));
	if (!scan_params) {
		return driver_error(BT_DRIVER, BT_ERR_NOT_ENOUGH_MEMORY, NULL);
	}

	scan_params->scan_type           = BLE_SCAN_TYPE_ACTIVE;
	scan_params->own_addr_type       = BLE_ADDR_TYPE_PUBLIC;
	scan_params->scan_filter_policy  = BLE_SCAN_FILTER_ALLOW_ALL;
	scan_params->scan_interval       = 0x50;
	scan_params->scan_window         = 0x3;

	esp_ble_gap_set_scan_params(scan_params);

	// Wait for scan start completion
	EventBits_t uxBits = xEventGroupWaitBits(bt_event, evBT_SCAN_START_COMPLETE | evBT_SCAN_START_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);
	if (uxBits & (evBT_SCAN_START_COMPLETE)) {
	} else if (uxBits & (evBT_SCAN_START_ERROR)) {
		free(scan_params);
		return driver_error(BT_DRIVER, BT_ERR_CANT_START_SCAN, NULL);
	}

	free(scan_params);
	return NULL;
}


static esp_ble_scan_params_t ble_scan_params = { // Agregado: Parámetros para scan LE
    .scan_type              = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50, // Scan interval. This is defined as the time interval from when the Controller started its last LE scan
                                    // until it begins the subsequent LE scan. Range: 0x0004 to 0x4000
                                    // Default: 0x0010 (10 ms) Time = N * 0.625 msec Time Range: 2.5 msec to 10.24 seconds
    //--                            //--
    .scan_window            = 0x30  // Scan window. The duration of the LE scan. LE_Scan_Window shall be less than or equal
                                    // to LE_Scan_Interval Range: 0x0004 to 0x4000 Default: 0x0010 (10 ms)
                                    // Time = N * 0.625 msec Time Range: 2.5 msec to 10240 msec
    //.scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};


// Agregado: setear duración de scan LE y BR/EDR
driver_error_t *bt_scan_set_le_duration(uint32_t ble_duration, uint16_t scan_interval, uint16_t scan_window) {
	ble_scan_duration             = ble_duration;
	ble_scan_params.scan_interval = scan_interval;
	ble_scan_params.scan_window   = scan_window;
	ble_scan_params_set           = false;
	return NULL;
}
driver_error_t *bt_scan_set_bredr_duration(uint8_t bredr_inq_len) {
    bredr_scan_inq_len = bredr_inq_len;
    return NULL;
}


// Agregado: Setear el valor mínimo de RSSI permitido para Scan LE y Discovery BR/EDR (0 es sin mínimo)
driver_error_t *bt_scan_set_min_rssi(int min_rssi) {
	scan_min_rssi = min_rssi;
	return NULL;
}


// Agregado: Comenzar escaneo LE
driver_error_t *bt_scan_start_le(bt_scan_callback_t cb, int cb_id) {
	// Sanity checks
	if (bt_mode_set != ESP_BT_MODE_BTDM && bt_mode_set != ESP_BT_MODE_BLE) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}
	
	// Store callback data
	callback    = cb;
	callback_id = cb_id;
	
	if (!ble_scan_params_set) {
		esp_ble_gap_set_scan_params(&ble_scan_params);
	} else {
		esp_ble_gap_start_scanning(ble_scan_duration);
	}
	
	// Wait for scan start completion
	EventBits_t uxBits = xEventGroupWaitBits(bt_event, evBT_SCAN_START_COMPLETE | evBT_SCAN_START_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);
	if (uxBits & (evBT_SCAN_START_COMPLETE)) {
	} else if (uxBits & (evBT_SCAN_START_ERROR)) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_START_SCAN, NULL);
	}
	
	return NULL;
}


// Agregado: Comenzar escaneo BR/EDR
driver_error_t *bt_scan_start_bredr(bt_scan_callback_t cb, int cb_id) {
	// Sanity checks
	if (bt_mode_set != ESP_BT_MODE_BTDM && bt_mode_set != ESP_BT_MODE_CLASSIC_BT) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}
	
	// Store callback data
	callback    = cb;
	callback_id = cb_id;
	
	/* start to discover nearby Bluetooth devices */
	esp_err_t error;
	if ((error = esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, bredr_scan_inq_len, 0))) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_START_SCAN, NULL);
	}
	return NULL;
}


// Sirve si se inició con bt_scan_start_le o con bt_scan_start
driver_error_t *bt_scan_stop() {
	// Sanity checks
	if (!setup) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}

	// Stop scan
	esp_ble_gap_stop_scanning();

	// Wait for scan stop completion
	EventBits_t uxBits = xEventGroupWaitBits(bt_event, evBT_SCAN_STOP_COMPLETE | evBT_SCAN_STOP_ERROR, pdTRUE, pdFALSE, portMAX_DELAY);
	if (uxBits & (evBT_SCAN_STOP_COMPLETE)) {
		return NULL;
	} else if (uxBits & (evBT_SCAN_STOP_ERROR)) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_STOP_SCAN, NULL);
	}

	return NULL;
}


// Agregado: Detener un Scan BR/EDR
driver_error_t *bt_scan_stop_bredr() {
	if (!setup) {
		return driver_error(BT_DRIVER, BT_ERR_IS_NOT_SETUP, NULL);
	}
	esp_err_t error;
	if ((error = esp_bt_gap_cancel_discovery())) {
		return driver_error(BT_DRIVER, BT_ERR_CANT_STOP_SCAN, NULL);
	}
	return NULL;
}


// Agregado: Indica si se está ejecutando un Scan LE (debe haberse iniciado con bt_scan_start_le)
driver_error_t *bt_is_le_scanning(bool *isLeScanning) {
	*isLeScanning = ble_scan_running;
	return NULL;
}


// Agregado: Indica si se está ejecutando un Scan BR/EDR
driver_error_t *bt_is_bredr_scanning(bool *isBrEdrScanning) {
	*isBrEdrScanning = bredr_scan_running;
	return NULL;
}


// Agregado: para obtener el BD_ADDR de este dispositivo
driver_error_t *bt_get_bdaddr(uint8_t *bd_addr) {
    if (!bd_addr)
		return driver_error(BT_DRIVER, BT_ERR_INVALID_ARGUMENT, NULL);
	// esp-idf/components/esp32/include/esp_system.h:
	//     esp_err_t esp_read_mac(uint8_t* mac, esp_mac_type_t type)
	//     esp_mac_type_t (enum): Tipo de MAC Address: ESP_MAC_WIFI_STA | ESP_MAC_WIFI_SOFTAP | ESP_MAC_BT | ESP_MAC_ETH
    esp_read_mac(bd_addr, ESP_MAC_BT);
	return NULL;
}


// Libera toda la memoria de Bluetooth. No se puede deshacer hasta el siguiente reinicio. Si se va a usar sólo uno de los modos (BLE o BR/EDR),
// al ejecutar bt_setup_dual, se libera la memoria del modo no utilizado.
// Precondición: No puede haber sido inicializado.
driver_error_t *bt_free_mem() {
	if (setup_dual) {
		esp_bluedroid_disable();
		esp_bluedroid_deinit();
		esp_bt_controller_disable();
		esp_bt_controller_deinit();
	}
	if (esp_bt_mem_release(ESP_BT_MODE_BTDM) == ESP_OK)
		printf("Bluetooth memory released\n");
	else
		printf("Error releasing Bluetooth memory\n");
	return NULL;
}

#endif
