/****************************************************************************
 * GATT CLIENT SINGLE
 * This file is for gatt client. It can scan ble device, connect one device.
 *
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "controller.h"

#include "bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "esp_task_wdt.h"

#include <sys/time.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

// ----------------------------------------- USER DEFINE---------------------------------------------------------------------------------- */
#define EX_UART_NUM UART_NUM_2
#define BUF_SIZE (1024)
#define TRACK_STOP 0x11
#define TRACK_START 0x0F
#define TRACK_FINAL 0xF0
#define NO_EXT 0x00
#define EXT_WORK 0x10
#define EXT_ENERGY 0x20
#define TAG_UART "uart_events"
#define ESP_SERVER_NUMBER 0x01
#define UART1_tx 17
#define UART1_rx 16
#define CONNECTING 0xAA
#define ALL_CONNECTED 0xFF
#define CONNECTION_TIMEOUT 0xEE

#define MASTER_COMMAND	0xFF
#define STBY 			0xAA
#define INACTION_TIME   10
// ----------------------------------------- USER DEFINE---------------------------------------------------------------------------------- */

static const char remote_device_name[] = "ESP_GATTS_DEMO";

static bool connect = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* eclare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event,
		esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
		esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event,
		esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
//static void uart_handler();

static esp_bt_uuid_t remote_filter_service_uuid = { .len = ESP_UUID_LEN_16,
		.uuid = { .uuid16 = REMOTE_SERVICE_UUID, }, };

static esp_bt_uuid_t remote_filter_char_uuid = { .len = ESP_UUID_LEN_16, .uuid =
		{ .uuid16 = REMOTE_NOTIFY_CHAR_UUID, }, };

static esp_bt_uuid_t notify_descr_uuid = { .len = ESP_UUID_LEN_16, .uuid = {
		.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG, }, };

static esp_ble_scan_params_t ble_scan_params = { .scan_type =
		BLE_SCAN_TYPE_ACTIVE, .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
		.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL, .scan_interval = 0x50,
		.scan_window = 0x30 };

struct gattc_profile_inst {
	esp_gattc_cb_t gattc_cb;
	uint16_t gattc_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_start_handle;
	uint16_t service_end_handle;
	uint16_t char_handle;
	esp_bd_addr_t remote_bda;
};

// ----------------------------------------- USER DATA ---------------------------------------------------------------------------------- */
typedef struct {
	uint8_t profile;
	uint8_t state;
	uint8_t level;
	uint8_t batt_level;
	uint8_t batt_stat;
	bool evt;
} moduleState;

typedef struct {
	esp_gatt_if_t gatt_if;
	uint16_t character_attr;
} conn_params;

moduleState state;
conn_params connection;

static QueueHandle_t uart0_queue;
static QueueHandle_t player_queue;

static bool conn_device_a = false;
static bool conn_device_b = false;
static bool conn_device_c = false;
static bool conn_device_d = false;
static bool connecting = false;
static bool scan_stop = false;
static bool last_message = false;
struct timeval start_scanning, scaning_time, now, receive;

// ----------------------------------------- USER DATA ---------------------------------------------------------------------------------- */

// ----------------------------------------- USER FNC  ---------------------------------------------------------------------------------- */

/*
 * moduleState Qstate;
 *
 * Zdefiniowac w obsludze notyfikacji
 * Qstate.state = ...
 * Qstate.level = ...
 *
 *
 */
static void player_task(void* arg) {
	uint8_t* data_tx = (uint8_t*) malloc(BUF_SIZE);
	uint8_t* data_rx = (uint8_t*) malloc(BUF_SIZE);
	uint8_t len_tx = 0;
	uint8_t actualProfile = 0x00;
	moduleState Qstat;
	bool restart = false;

	//connecting = true;

	while (1) {
		if (conn_device_a && conn_device_b && conn_device_c && conn_device_d) {

			printf("\n ----------------------- \n ------------------- \n ");

			// ODBIOR WIADOMOSCI OD DEKODERA -------------------------------------------------
			int len = uart_read_bytes(EX_UART_NUM, data_rx, BUF_SIZE,
					100 / portTICK_RATE_MS);
			if (len == 2) {
				ESP_LOGI(TAG_UART, "uart read : %d", len);
				if (data_rx[0] == TRACK_FINAL) {
					if (connecting == true) {
						data_tx[0] = TRACK_START;
						data_tx[1] = CONNECTING;
					} else {
						data_tx[0] = TRACK_START;
						data_tx[1] = data_rx[1];
					}
					len_tx = 2;
				}
			}
			// ODBIOR WIADOMOSCI OD DEKODERA -------------------------------------------------

			if (connecting == true) {
				data_tx[0] = TRACK_START;
				data_tx[1] = ALL_CONNECTED;
				len_tx = 2;

				connecting = false;
			}

			if (xQueueReceive(player_queue, &Qstat, portMAX_DELAY)) {
				if (Qstat.state != 0x00) {
					/*if(Qstat.level == NO_EXT){
					 data[0] = TRACK_START;
					 data[1] = Qstat.profile;
					 }else if((Qstat.level == EXT_WORK) || (Qstat.level == EXT_ENERGY)){
					 data[0] = TRACK_START;
					 data[1] = ((Qstat.level) || (Qstat.profile));
					 }*/
					data_tx[0] = TRACK_START;
					data_tx[1] = ((Qstat.level) || (Qstat.profile));
					len_tx = 2;
				} else if (Qstat.state == 0x00) {
					if (Qstat.profile == actualProfile) {
						data_tx[0] = TRACK_STOP;
						data_tx[1] = 0x00;
						len_tx = 2;
						actualProfile = 0x00;
						last_message = true;
						gettimeofday(&receive, NULL);
					} else {

					}
				}
				actualProfile = Qstat.profile;

				if (Qstat.batt_stat == 0xFF) {
					/* ----- TUTAJ WYMUSZANIE STANU UŚPIENIA ----- */
				}
			}

		} else if (!(conn_device_a && conn_device_b && conn_device_c
				&& conn_device_d)) {
			// DODAC ZMIENNA MOWIACA O LACZENIU Z INNYMI URZADZENIAMI

			if (scan_stop == false) {
				int len = uart_read_bytes(EX_UART_NUM, data_rx, BUF_SIZE,
						100 / portTICK_RATE_MS);
				if (len == 2) {
					ESP_LOGI(TAG_UART, "uart read : %d", len);
					if (data_rx[0] == TRACK_FINAL) {
						if (connecting == true) {
							data_tx[0] = TRACK_START;
							data_tx[1] = CONNECTING;
						} else {
							data_tx[0] = TRACK_START;
							data_tx[1] = data_rx[1];
						}
						len_tx = 2;
					}

				}

				if (data_tx[1] != CONNECTING) {
					data_tx[0] = TRACK_START;
					data_tx[1] = CONNECTING;
					len_tx = 2;

					connecting = true;
				}
			}

			gettimeofday(&scaning_time, NULL);
			if ((scaning_time.tv_sec - start_scanning.tv_sec) > 30) {
				data_tx[0] = TRACK_START;
				data_tx[1] = CONNECTION_TIMEOUT;
				len_tx = 2;
				restart = true;
			}

		}

		// WYSLANIE WIADOMOSCI DO DEKODERA -----------------------------------------------

		if (len_tx > 0) {
			printf("\nTO PLAYER:\ncommand:  %d\ndata: %d\nactual profile: %d\n",
					data_tx[0], data_tx[1], actualProfile);
			uart_write_bytes(EX_UART_NUM, (const char*) data_tx, len_tx);
			len_tx = 0;
			//connecting = true;
			/*if (data_tx[0] == CONNECTING) {
			 vTaskDelay(3000 / portTICK_PERIOD_MS);
			 }*/

		}

		if (restart == true) {
			vTaskDelay(10000 / portTICK_PERIOD_MS);
			printf("ESP Restart now\n");
			restart = false;
			esp_restart();
		}

		// WYSLANIE WIADOMOSCI DO DEKODERA -----------------------------------------------
		vTaskDelay(10 / portTICK_PERIOD_MS);
		// ZEROWANIE WATCHDOGA ----------- -----------------------------------------------
		//esp_task_wdt_feed();
		// ZEROWANIE WATCHDOGA ----------- -----------------------------------------------

	}
}

static void uart_event_task(void *pvParameters) {
	uart_event_t event;
	size_t buffered_size;
	uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
	for (;;) {
		//Waiting for UART event.
		if (xQueueReceive(uart0_queue, (void * )&event,
				(portTickType)portMAX_DELAY)) {
			ESP_LOGI(TAG_UART, "uart[%d] event:", EX_UART_NUM);
			switch (event.type) {
			//Event of UART receving data
			/*We'd better handler data event fast, there would be much more data events than
			 other types of events. If we take too much time on data event, the queue might
			 be full.
			 in this example, we don't process data in event, but read data outside.*/
			case UART_DATA:
				uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
				ESP_LOGI(TAG_UART, "data, len: %d; buffered len: %d",
						event.size, buffered_size)
				;
				break;
				//Event of HW FIFO overflow detected
			case UART_FIFO_OVF:
				ESP_LOGI(TAG_UART, "hw fifo overflow\n")
				;
				//If fifo overflow happened, you should consider adding flow control for your application.
				//We can read data out out the buffer, or directly flush the rx buffer.
				uart_flush(EX_UART_NUM);
				break;
				//Event of UART ring buffer full
			case UART_BUFFER_FULL:
				ESP_LOGI(TAG_UART, "ring buffer full\n")
				;
				//If buffer full happened, you should consider encreasing your buffer size
				//We can read data out out the buffer, or directly flush the rx buffer.
				uart_flush(EX_UART_NUM);
				break;
				//Event of UART RX break detected
			case UART_BREAK:
				ESP_LOGI(TAG_UART, "uart rx break\n")
				;
				break;
				//Event of UART parity check error
			case UART_PARITY_ERR:
				ESP_LOGI(TAG_UART, "uart parity error\n")
				;
				break;
				//Event of UART frame error
			case UART_FRAME_ERR:
				ESP_LOGI(TAG_UART, "uart frame error\n")
				;
				break;
				//UART_PATTERN_DET
			case UART_PATTERN_DET:
				ESP_LOGI(TAG_UART, "uart pattern detected\n")
				;
				break;
				//Others
			default:
				ESP_LOGI(TAG_UART, "uart event type: %d\n", event.type)
				;
				break;
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);
}

static void uart_init(void) {
	uart_config_t uart_config = { .baud_rate = 9600, .data_bits =
			UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 122, };
	//Set UART parameters
	uart_param_config(EX_UART_NUM, &uart_config);
	//Set UART log level
	esp_log_level_set(TAG_UART, ESP_LOG_INFO);
	//Install UART driver, and get the queue.
	uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10,
			&uart0_queue, 0);

	//Set UART pins (using UART0 default pins ie no changes.)
	uart_set_pin(EX_UART_NUM, UART1_tx, UART1_rx, UART_PIN_NO_CHANGE,
	UART_PIN_NO_CHANGE);

	//Set uart pattern detect function.
	uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10);

	uart_enable_rx_intr(EX_UART_NUM);

	//xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

	//uart_isr_register(EX_UART_NUM, uart_handler, void * arg, int intr_alloc_flags,  uart_isr_handle_t *handle);
}

// ----------------------------------------- USER FNC  ---------------------------------------------------------------------------------- */

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
		[PROFILE_A_APP_ID] = { .gattc_cb = gattc_profile_event_handler,
				.gattc_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		}, };

static void gattc_profile_event_handler(esp_gattc_cb_event_t event,
		esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
	esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *) param;

	switch (event) {
	case ESP_GATTC_REG_EVT:
		ESP_LOGI(GATTC_TAG, "REG_EVT")
		;
		esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
		if (scan_ret) {
			ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x",
					scan_ret);
		}
		break;
	case ESP_GATTC_CONNECT_EVT: {
		//p_data->connect.status always be ESP_GATT_OK
		ESP_LOGI(GATTC_TAG,
				"ESP_GATTC_CONNECT_EVT conn_id %d, if %d, status %d",
				p_data->connect.conn_id, gattc_if, p_data->connect.status);
		gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
		memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
				p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
		ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
		esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
		esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if,
				p_data->connect.conn_id);
		if (mtu_ret) {
			ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
		}

		connection.gatt_if = gattc_if;

		break;
	}
	case ESP_GATTC_OPEN_EVT:
		if (param->open.status != ESP_GATT_OK) {
			ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
			break;
		}
		ESP_LOGI(GATTC_TAG, "open success")
		;
		break;
	case ESP_GATTC_CFG_MTU_EVT:
		if (param->cfg_mtu.status != ESP_GATT_OK) {
			ESP_LOGE(GATTC_TAG, "config mtu failed, error status = %x",
					param->cfg_mtu.status);
		}
		ESP_LOGI(GATTC_TAG,
				"ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d",
				param->cfg_mtu.status, param->cfg_mtu.mtu,
				param->cfg_mtu.conn_id)
		;
		esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id,
				&remote_filter_service_uuid);
		break;
	case ESP_GATTC_SEARCH_RES_EVT: {
		ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_RES_EVT");
		esp_gatt_srvc_id_t *srvc_id =
				(esp_gatt_srvc_id_t *) &p_data->search_res.srvc_id;
		if (srvc_id->id.uuid.len == ESP_UUID_LEN_16
				&& srvc_id->id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
			ESP_LOGI(GATTC_TAG, "service found");
			get_server = true;
			gl_profile_tab[PROFILE_A_APP_ID].service_start_handle =
					p_data->search_res.start_handle;
			gl_profile_tab[PROFILE_A_APP_ID].service_end_handle =
					p_data->search_res.end_handle;
			ESP_LOGI(GATTC_TAG, "UUID16: %x", srvc_id->id.uuid.uuid.uuid16);
		}
		break;
	}
	case ESP_GATTC_SEARCH_CMPL_EVT:
		if (p_data->search_cmpl.status != ESP_GATT_OK) {
			ESP_LOGE(GATTC_TAG, "search service failed, error status = %x",
					p_data->search_cmpl.status);
			break;
		}
		ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT")
		;
		if (get_server) {
			uint16_t count = 0;
			esp_gatt_status_t status = esp_ble_gattc_get_attr_count(gattc_if,
					p_data->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
					gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
					gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
					INVALID_HANDLE, &count);
			if (status != ESP_GATT_OK) {
				ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
			}

			if (count > 0) {
				char_elem_result = (esp_gattc_char_elem_t *) malloc(
						sizeof(char_elem_result) * count);
				if (!char_elem_result) {
					ESP_LOGE(GATTC_TAG, "gattc no mem");
				} else {
					status =
							esp_ble_gattc_get_char_by_uuid(gattc_if,
									p_data->search_cmpl.conn_id,
									gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
									gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
									remote_filter_char_uuid, char_elem_result,
									&count);
					if (status != ESP_GATT_OK) {
						ESP_LOGE(GATTC_TAG,
								"esp_ble_gattc_get_char_by_uuid error");
					}

					/*  Every service have only one char in our 'ESP_GATTS_DEMO' demo, so we used first 'char_elem_result' */
					if (count > 0
							&& (char_elem_result[0].properties
									& ESP_GATT_CHAR_PROP_BIT_NOTIFY)) {
						gl_profile_tab[PROFILE_A_APP_ID].char_handle =
								char_elem_result[0].char_handle;
						esp_ble_gattc_register_for_notify(gattc_if,
								gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
								char_elem_result[0].char_handle);
					}
				}
				/* free char_elem_result */
				free(char_elem_result);
			} else {
				ESP_LOGE(GATTC_TAG, "no char found");
			}
		}
		break;
	case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
		ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");
		if (p_data->reg_for_notify.status != ESP_GATT_OK) {
			ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY failed: error status = %d",
					p_data->reg_for_notify.status);
		} else {
			uint16_t count = 0;
			uint16_t notify_en = 1;
			esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(
					gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
					ESP_GATT_DB_DESCRIPTOR,
					gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
					gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
					gl_profile_tab[PROFILE_A_APP_ID].char_handle, &count);
			if (ret_status != ESP_GATT_OK) {
				ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
			}
			if (count > 0) {
				descr_elem_result = malloc(sizeof(descr_elem_result) * count);
				if (!descr_elem_result) {
					ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
				} else {
					ret_status = esp_ble_gattc_get_descr_by_char_handle(
							gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id,
							p_data->reg_for_notify.handle, notify_descr_uuid,
							descr_elem_result, &count);
					if (ret_status != ESP_GATT_OK) {
						ESP_LOGE(GATTC_TAG,
								"esp_ble_gattc_get_descr_by_char_handle error");
					}

					/* Erery char have only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
					if (count
							> 0&& descr_elem_result[0].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[0].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG) {
						ret_status = esp_ble_gattc_write_char_descr(gattc_if,
								gl_profile_tab[PROFILE_A_APP_ID].conn_id,
								descr_elem_result[0].handle, sizeof(notify_en),
								(uint8_t *) &notify_en, ESP_GATT_WRITE_TYPE_RSP,
								ESP_GATT_AUTH_REQ_NONE);
					}

					if (ret_status != ESP_GATT_OK) {
						ESP_LOGE(GATTC_TAG,
								"esp_ble_gattc_write_char_descr error");
					}

					/* free descr_elem_result */
					free(descr_elem_result);
				}
			} else {
				ESP_LOGE(GATTC_TAG, "decsr not found");
			}

		}
		break;
	}
	case ESP_GATTC_NOTIFY_EVT:
		ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:")
		;
		esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);

		/* *************************************************************************** */
		state.profile = ESP_SERVER_NUMBER; //0x01 DLA PROFILU A
		state.state = p_data->notify.value[0];
		state.level = p_data->notify.value[1];
		//state.batt_level = (((0x0000 | (p_data->notify.value[2])) << 8)
		//		| (p_data->notify.value[3]));
		state.batt_level = 0x00;
		state.batt_stat = 0x00;
		//gettimeofday(&receive, NULL);
		xQueueSendFromISR(player_queue, &state, NULL);
		/* *************************************************************************** */

		break;
	case ESP_GATTC_WRITE_DESCR_EVT: /* CLIENT TO SERVER COMMUNICATION */
		if (p_data->write.status != ESP_GATT_OK) {
			ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x",
					p_data->write.status);
			break;
		}
		ESP_LOGI(GATTC_TAG, "write descr success ")
		;
		uint8_t write_char_data[] = { 0x01, 0x0F, 0xF0, 0x10 };
		esp_ble_gattc_write_char(gattc_if,
				gl_profile_tab[PROFILE_A_APP_ID].conn_id,
				gl_profile_tab[PROFILE_A_APP_ID].char_handle,
				sizeof(write_char_data), write_char_data,
				ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
		break;
	case ESP_GATTC_SRVC_CHG_EVT: {
		esp_bd_addr_t bda;
		memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
		ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
		esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
		break;
	}
	case ESP_GATTC_WRITE_CHAR_EVT:
		if (p_data->write.status != ESP_GATT_OK) {
			ESP_LOGE(GATTC_TAG, "write char failed, error status = %x",
					p_data->write.status);
			break;
		}
		ESP_LOGI(GATTC_TAG, "write char success ")
		;
		break;
	case ESP_GATTC_DISCONNECT_EVT:
		connect = false;
		get_server = false;
		ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, status = %d",
				p_data->disconnect.status)
		;
		ESP_LOGI(GATTC_TAG, "ESP_RESTART")
		;
		/* -------------------------------------- RESTART ESP  -------------------------------------- */
		esp_restart();
		break;
	default:
		break;
	}
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event,
		esp_ble_gap_cb_param_t *param) {
	uint8_t *adv_name = NULL;
	uint8_t adv_name_len = 0;
	switch (event) {
	case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
		//the unit of the duration is second
		gettimeofday(&start_scanning, NULL);
		uint32_t duration = 30;
		esp_ble_gap_start_scanning(duration);
		break;
	}
	case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
		//scan start complete event to indicate scan start successfully or failed
		if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x",
					param->scan_start_cmpl.status);
			break;
		}
		ESP_LOGI(GATTC_TAG, "scan start success")
		;

		break;
	case ESP_GAP_BLE_SCAN_RESULT_EVT: {
		esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *) param;
		switch (scan_result->scan_rst.search_evt) {
		case ESP_GAP_SEARCH_INQ_RES_EVT:
			esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
			ESP_LOGI(GATTC_TAG,
					"searched Adv Data Len %d, Scan Response Len %d",
					scan_result->scan_rst.adv_data_len,
					scan_result->scan_rst.scan_rsp_len)
			;
			adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
					ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
			ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len)
			;
			esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
			ESP_LOGI(GATTC_TAG, "\n")
			;
			if (adv_name != NULL) {
				if (strlen(remote_device_name) == adv_name_len
						&& strncmp((char *) adv_name, remote_device_name,
								adv_name_len) == 0) {
					ESP_LOGI(GATTC_TAG, "searched device %s\n",
							remote_device_name);
					if (connect == false) {
						connect = true;
						conn_device_a = true;
						conn_device_b = true;
						conn_device_c = true;
						conn_device_d = true;
						ESP_LOGI(GATTC_TAG, "connect to the remote device.");
						printf("\n\n\n\nSTOP SCANNING\n\n\n\n");
						esp_ble_gap_stop_scanning();
						scan_stop = true;
						esp_ble_gattc_open(
								gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
								scan_result->scan_rst.bda, true);
					}
				}
			}
			break;
		case ESP_GAP_SEARCH_INQ_CMPL_EVT:
			break;
		default:
			break;
		}
		break;
	}

	case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
		if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x",
					param->scan_stop_cmpl.status);
			break;
		}
		ESP_LOGI(GATTC_TAG, "stop scan successfully")
		;
		scan_stop = true;
		break;

	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x",
					param->adv_stop_cmpl.status);
			break;
		}
		ESP_LOGI(GATTC_TAG, "stop adv successfully")
		;
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(GATTC_TAG,
				"update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				param->update_conn_params.status,
				param->update_conn_params.min_int,
				param->update_conn_params.max_int,
				param->update_conn_params.conn_int,
				param->update_conn_params.latency,
				param->update_conn_params.timeout)
		;
		break;
	default:
		break;
	}
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
		esp_ble_gattc_cb_param_t *param) {
	/* If event is register event, store the gattc_if for each profile */
	if (event == ESP_GATTC_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
		} else {
			ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
					param->reg.app_id, param->reg.status);
			return;
		}
	}

	/* If the gattc_if equal to profile A, call profile A cb handler,
	 * so here call each profile's callback */
	do {
		int idx;
		for (idx = 0; idx < PROFILE_NUM; idx++) {
			if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
			gattc_if == gl_profile_tab[idx].gattc_if) {
				if (gl_profile_tab[idx].gattc_cb) {
					gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
				}
			}
		}
	} while (0);
}

void app_main() {
	// Initialize NVS.
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()
	;
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTC_TAG,
				"%s initialize controller failed, error code = %x\n", __func__,
				ret);
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
	if (ret) {
		ESP_LOGE(GATTC_TAG, "%s enable controller failed, error code = %x\n",
				__func__, ret);
		return;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTC_TAG, "%s init bluetooth failed, error code = %x\n",
				__func__, ret);
		return;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed, error code = %x\n",
				__func__, ret);
		return;
	}

	//register the  callback function to the gap module
	ret = esp_ble_gap_register_callback(esp_gap_cb);
	if (ret) {
		ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x\n",
				__func__, ret);
		return;
	}

	//register the callback function to the gattc module
	ret = esp_ble_gattc_register_callback(esp_gattc_cb);
	if (ret) {
		ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x\n",
				__func__, ret);
		return;
	}

	ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
	if (ret) {
		ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x\n",
				__func__, ret);
	}
	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret) {
		ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x",
				local_mtu_ret);
	}
	uart_init();

	player_queue = xQueueCreate(10, sizeof(uint32_t));
	xTaskCreate(player_task, "player_task", 2048, NULL, 10, NULL);

	/*while (conn_device_a == false){
	 vTaskDelay (500 / portTICK_PERIOD_MS);
	 printf ("\nNO DEVICE");
	 };*/
	do {
		if ((last_message == true) && (conn_device_a == true)) {
			//printf("\nlast message");
			gettimeofday(&now, NULL);
			if ((now.tv_sec - receive.tv_sec) > INACTION_TIME) {
				uint8_t write_char_data[] = { MASTER_COMMAND, 0x0F, 0xF0, STBY };
				esp_ble_gattc_write_char(connection.gatt_if,
						gl_profile_tab[PROFILE_A_APP_ID].conn_id,
						gl_profile_tab[PROFILE_A_APP_ID].char_handle,
						sizeof(write_char_data), write_char_data,
						ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
				printf("\nSEND STANDBY REQUEST");
				last_message = false;
			}
		}
	} while (1);
}

