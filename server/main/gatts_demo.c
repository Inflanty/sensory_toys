#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt.h"

#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"

#define GATTS_TAG "GATTS_DEMO" /* Nzawa urzadzenia w komunikacji monitor*/

/*Declare the static function*/
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

// ----------------------------------------- DEFINES ------------------------------------------------------------------------------------ */

#define GATTS_SERVICE_UUID_TEST_A   0x00FF//0x1815	/*AUTOMATION IO*/
#define GATTS_CHAR_UUID_TEST_A      0xFF01//0x2A3F	/*ALERT STATUS*/
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

#define GATTS_SERVICE_UUID_TEST_B  	0x00EE//0x180A	/*Device information*/
#define GATTS_CHAR_UUID_TEST_B      0xEE01//0x2A3F	/*Alert status*/
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define TEST_DEVICE_NAME            "ESP_GATTS_DEMO"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

#define PREPARE_BUF_MAX_SIZE 1024

#define GPIO_OUTPUT_IO_0    	18
#define GPIO_OUTPUT_PIN_SEL  	(1<<GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_IO_0     	4
#define GPIO_INPUT_IO_1     	5
#define GPIO_INPUT_PIN_SEL  	((1<<GPIO_INPUT_IO_0) | (1<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 	0

#define IMMOBILITY 			0x00
#define MOTION 				0x01
#define EXP_CONNECT		 	0x02
#define EXP_DISCONNECT		0x00

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_GROUP    TIMER_GROUP_0     /*!< Test on timer group 0 */
#define TIMER_DIVIDER   16               /*!< Hardware timer clock divider */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (3.4179)   /*!< test interval for timer 0 */
#define TIMER_INTERVAL1_SEC   (4)   /*!< test interval for timer 1 */
#define TEST_WITHOUT_RELOAD   0   /*!< example of auto-reload mode */
#define TEST_WITH_RELOAD   1      /*!< example without auto-reload mode */

// ----------------------------------------- DEFINES ------------------------------------------------------------------------------------ */

// ----------------------------------------- BLE DATA ----------------------------------------------------------------------------------- */

uint8_t char1_str[] = { 0x11, 0x22, 0x33 };
esp_gatt_char_prop_t a_property = 0;
esp_gatt_char_prop_t b_property = 0;

esp_attr_value_t gatts_demo_char1_val = { .attr_max_len =
GATTS_DEMO_CHAR_VAL_LEN_MAX, .attr_len = sizeof(char1_str), .attr_value =
		char1_str, };

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
	0x02, 0x01, 0x06,
	0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
	0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
	0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
/* LSB <--------------------------------------------------------------------------------> MSB */
/*first uuid, 16bit, [12],[13] is the value*/
0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE,
		0x00, 0x00, 0x00,
		/*second uuid, 32bit, [12], [13], [14], [15] is the value*/
		0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00,
		0xFF, 0x00, 0x00, 0x00, };

/* The length of adv data must be less than 31 bytes
 //static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
 adv data*/
static esp_ble_adv_data_t adv_data = { .set_scan_rsp = false, .include_name =
true, .include_txpower = true, .min_interval = 0x20, .max_interval = 0x40,
		.appearance = 0x00, .manufacturer_len = 0, /*TEST_MANUFACTURER_DATA_LEN,*/
		.p_manufacturer_data = NULL, /*&test_manufacturer[0],*/
		.service_data_len = 0, .p_service_data = NULL, .service_uuid_len = 32,
		.p_service_uuid = adv_service_uuid128, .flag =
				(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), };
/* scan response data*/
static esp_ble_adv_data_t scan_rsp_data = { .set_scan_rsp = true,
		.include_name = true, .include_txpower = true, .min_interval = 0x20,
		.max_interval = 0x40, .appearance = 0x00, .manufacturer_len = 0, /*TEST_MANUFACTURER_DATA_LEN,*/
		.p_manufacturer_data = NULL, /*&test_manufacturer[0],*/
		.service_data_len = 0, .p_service_data = NULL, .service_uuid_len = 32,
		.p_service_uuid = adv_service_uuid128, .flag =
				(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT), };

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = { .adv_int_min = 0x20, .adv_int_max =
		0x40, .adv_type = ADV_TYPE_IND, .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
/*.peer_addr            =*/
/*.peer_addr_type       =*/
.channel_map = ADV_CHNL_ALL, .adv_filter_policy =
		ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, };

struct gatts_profile_inst {
	esp_gatts_cb_t gatts_cb;
	uint16_t gatts_if;
	uint16_t app_id;
	uint16_t conn_id;
	uint16_t service_handle;
	esp_gatt_srvc_id_t service_id;
	uint16_t char_handle;
	esp_bt_uuid_t char_uuid;
	esp_gatt_perm_t perm;
	esp_gatt_char_prop_t property;
	uint16_t descr_handle;
	esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
		[PROFILE_A_APP_ID] = { .gatts_cb = gatts_profile_a_event_handler,
				.gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		}, [PROFILE_B_APP_ID] = { .gatts_cb = gatts_profile_b_event_handler, /* This demo does not implement, similar as profile A */
		.gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
		}, };

typedef struct {
	uint8_t *prepare_buf;
	int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;
static prepare_type_env_t b_prepare_write_env;

// ----------------------------------------- BLE DATA ----------------------------------------------------------------------------------- */

// ----------------------------------------- USER DATA ---------------------------------------------------------------------------------- */
typedef struct {
	uint8_t state;
	uint8_t level;
	uint16_t batt_level;
	bool charger;
	bool evt;
} moduleState;

typedef struct {
	int type; /*!< event type */
	int group; /*!< timer group */
	int idx; /*!< timer number */
	uint64_t counter_val; /*!< timer counter value */
} timer_event_t;

typedef struct {
	esp_gatt_if_t gatt_if;
	uint16_t character_attr;
} conn_params;

moduleState module;

conn_params connection;

volatile int timer_int = 0;

volatile int motion_status = 1;
//
// ----------------------------------------- USER DATA ---------------------------------------------------------------------------------- */

// ----------------------------------------- FUNCTION DEF ------------------------------------------------------------------------------- */

void example_write_event_env(esp_gatt_if_t gatts_if,
		prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env,
		esp_ble_gatts_cb_param_t *param);
void notify_data(esp_gatt_if_t gatt_server_if, uint16_t attr_handle,
		uint8_t *value, uint8_t length);

// ----------------------------------------- FUNCTION DEF ------------------------------------------------------------------------------- */

// ----------------------------------------- FUNCTIONS ---------------------------------------------------------------------------------- */
/*
 static void inline print_u64(uint64_t val)
 {
 printf("0x%08x%08x\n", (uint32_t) (val >> 32), (uint32_t) (val));
 }
 */

static xQueueHandle gpio_evt_queue = NULL;
xQueueHandle timer_queue;

static void vibro_task(void* arg) {
	uint32_t io_num;

	module.state = IMMOBILITY;
	module.level = EXP_DISCONNECT;
	module.evt = false;

	timer_group_t group_num = TIMER_GROUP_0;
	timer_idx_t timer_num = TIMER_1;
	uint64_t load_val = 0x00000000ULL;
	uint64_t counter_val = 0;

	for (;;) {
		if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
			if (io_num == GPIO_INPUT_IO_0) {
				module.evt = true;
				timer_get_counter_value(group_num, timer_num, &counter_val);
				if (counter_val != 0x00000000ULL) {
					timer_pause(group_num, timer_num);
					if (timer_set_counter_value(group_num, timer_num,
							load_val) == ESP_OK) {
						if (timer_start(group_num, timer_num) == ESP_OK) {
							module.state = MOTION;
							printf("Timer_start find motion\n");
							if (TIMERG0.hw_timer[timer_num].config.alarm_en
									== 1) {

							} else {
								TIMERG0.hw_timer[timer_num].config.alarm_en = 1;
							}
						}
					}
				} else if (timer_start(group_num, timer_num) == ESP_OK) {
					module.state = MOTION;
					printf("Timer_start find motion\n");
					if (TIMERG0.hw_timer[timer_num].config.alarm_en == 1) {

					} else {
						TIMERG0.hw_timer[timer_num].config.alarm_en = 1;
					}
				}
			}
		} else if (io_num == GPIO_INPUT_IO_1) {
			/* mozna ustawic polaczenie z rozszezeniem */
		}

		/*if (timer_int == 1) {

		 module.evt = true;
		 if (module.state == MOTION) {
		 module.state = IMMOBILITY;
		 }
		 timer_int = 0;

		 }*/

		if ((module.evt == true) && (motion_status == 1)) {

			uint8_t notify_tab[4];
			notify_tab[0] = module.state;
			notify_tab[1] = module.level;
			notify_tab[2] = 0x00;
			notify_tab[3] = 0x00;

			//the size of notify_data[] need less than MTU size
			esp_ble_gatts_send_indicate(connection.gatt_if, 0,
					connection.character_attr, sizeof(notify_tab), notify_tab,
					false);

			module.evt = false;
			motion_status = 0;

			printf(
					"\n-----------------------\nNotify send, send value = %d | %d | %d | %d\n-----------------------\n",
					notify_tab[0], notify_tab[1], notify_tab[2], notify_tab[3]);

		}

	}
}

void IRAM_ATTR timer_group0_isr(void *para) {

	timer_event_t evt;
	int timer_idx = (int) para;
	uint32_t intr_status = TIMERG0.int_st_timers.val;
	if ((intr_status & BIT(timer_idx)) && timer_idx == TIMER_1) {
		/*Timer1 is an example that will reload counter value*/
		TIMERG0.hw_timer[timer_idx].update = 1;
		/*We don't call a API here because they are not declared with IRAM_ATTR*/
		TIMERG0.int_clr_timers.t1 = 1;
		uint64_t timer_val = ((uint64_t) TIMERG0.hw_timer[timer_idx].cnt_high)
				<< 32 | TIMERG0.hw_timer[timer_idx].cnt_low;
		/*Post an event to out example task*/
		evt.type = TEST_WITH_RELOAD;
		evt.group = 0;
		evt.idx = timer_idx;
		evt.counter_val = timer_val;
		xQueueSendFromISR(timer_queue, &evt, NULL);
		/*For a auto-reload timer, we still need to set alarm_en bit if we want to enable alarm again.*/
		//TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
	}
}

static void timer_example_evt_task(void *arg) {
	while (1) {
		timer_event_t evt;
		xQueueReceive(timer_queue, &evt, portMAX_DELAY);
		if (evt.idx == TIMER_0) {

		} else if (evt.idx == TIMER_1) {
			module.evt = true;
			if (module.state != IMMOBILITY) {
				module.state = IMMOBILITY;

				if (module.evt == true) {

					uint8_t notify_tab[4];
					notify_tab[0] = module.state;
					notify_tab[1] = module.level;
					notify_tab[2] = 0x00;
					notify_tab[3] = 0x00;

					//the size of notify_data[] need less than MTU size
					esp_ble_gatts_send_indicate(connection.gatt_if, 0,
							connection.character_attr, sizeof(notify_tab),
							notify_tab,
							false);

					module.evt = false;
					motion_status = 1;
					timer_set_counter_value(TIMER_GROUP_0, TIMER_1,
							0x00000000ULL);

					printf(
							"\n-----------------------\nNotify send, send value = %d | %d | %d | %d\n-----------------------\n",
							notify_tab[0], notify_tab[1], notify_tab[2],
							notify_tab[3]);

				}

			}

		}

	}
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
		esp_ble_gap_cb_param_t *param) {
	switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
	adv_config_done &= (~adv_config_flag);
	if (adv_config_done==0) {
		esp_ble_gap_start_advertising(&adv_params);
	}
	break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
	adv_config_done &= (~scan_rsp_config_flag);
	if (adv_config_done==0) {
		esp_ble_gap_start_advertising(&adv_params);
	}
	break;
#else
	case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~adv_config_flag);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
	case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
		adv_config_done &= (~scan_rsp_config_flag);
		if (adv_config_done == 0) {
			esp_ble_gap_start_advertising(&adv_params);
		}
		break;
#endif
	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		/*advertising start complete event to indicate advertising start successfully or failed*/
		if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
		}
		break;
	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
			ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
		} else {
			ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
		}
		break;
	case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
		ESP_LOGI(GATTS_TAG,
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

void example_write_event_env(esp_gatt_if_t gatts_if,
		prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param) {
	esp_gatt_status_t status = ESP_GATT_OK;
	if (param->write.need_rsp) {
		if (param->write.is_prep) {
			if (prepare_write_env->prepare_buf == NULL) {
				prepare_write_env->prepare_buf = (uint8_t *) malloc(
				PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
				prepare_write_env->prepare_len = 0;
				if (prepare_write_env->prepare_buf == NULL) {
					ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem\n");
					status = ESP_GATT_NO_RESOURCES;
				}
			} else {
				if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
					status = ESP_GATT_INVALID_OFFSET;
				} else if ((param->write.offset + param->write.len)
						> PREPARE_BUF_MAX_SIZE) {
					status = ESP_GATT_INVALID_ATTR_LEN;
				}
			}

			esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *) malloc(
					sizeof(esp_gatt_rsp_t));
			gatt_rsp->attr_value.len = param->write.len;
			gatt_rsp->attr_value.handle = param->write.handle;
			gatt_rsp->attr_value.offset = param->write.offset;
			gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
			memcpy(gatt_rsp->attr_value.value, param->write.value,
					param->write.len);
			esp_err_t response_err = esp_ble_gatts_send_response(gatts_if,
					param->write.conn_id, param->write.trans_id, status,
					gatt_rsp);
			if (response_err != ESP_OK) {
				ESP_LOGE(GATTS_TAG, "Send response error\n");
			}
			free(gatt_rsp);
			if (status != ESP_GATT_OK) {
				return;
			}
			memcpy(prepare_write_env->prepare_buf + param->write.offset,
					param->write.value, param->write.len);
			prepare_write_env->prepare_len += param->write.len;

		} else {
			esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
					param->write.trans_id, status, NULL);
		}
	}
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env,
		esp_ble_gatts_cb_param_t *param) {
	if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
		esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
	} else {
		ESP_LOGI(GATTS_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
	}
	if (prepare_write_env->prepare_buf) {
		free(prepare_write_env->prepare_buf);
		prepare_write_env->prepare_buf = NULL;
	}
	prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {

	switch (event) {
	case ESP_GATTS_REG_EVT:
		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n",
				param->reg.status, param->reg.app_id)
		;
		gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
		gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len =
		ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 =
		GATTS_SERVICE_UUID_TEST_A;

		esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
#ifdef CONFIG_SET_RAW_ADV_DATA
		esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
		if (raw_adv_ret) {
			ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
		}
		adv_config_done |= adv_config_flag;
		esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
		if (raw_scan_ret) {
			ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
		}
		adv_config_done |= scan_rsp_config_flag;
#else
		//config adv data
		esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
		if (ret) {
			ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
		}
		adv_config_done |= adv_config_flag;
		//config scan response data
		ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
		if (ret) {
			ESP_LOGE(GATTS_TAG,
					"config scan response data failed, error code = %x", ret);
		}
		adv_config_done |= scan_rsp_config_flag;

#endif
		esp_ble_gatts_create_service(gatts_if,
				&gl_profile_tab[PROFILE_A_APP_ID].service_id,
				GATTS_NUM_HANDLE_TEST_A);

		connection.gatt_if = gatts_if;
		//connection.character_attr = gl_profile_tab[PROFILE_A_APP_ID].char_handle;

		break;
	case ESP_GATTS_READ_EVT: {
		/* **************************** Param init **************************** */
		ESP_LOGI(GATTS_TAG,
				"GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n",
				param->read.conn_id, param->read.trans_id, param->read.handle);
		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = 4;
		rsp.attr_value.value[0] = 0xde;
		rsp.attr_value.value[1] = 0xed;
		rsp.attr_value.value[2] = 0xbe;
		rsp.attr_value.value[3] = 0xef;
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
				param->read.trans_id, ESP_GATT_OK, &rsp);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG,
				"GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d",
				param->write.conn_id, param->write.trans_id,
				param->write.handle);
		if (!param->write.is_prep) {
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :",
					param->write.len);
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle
					== param->write.handle && param->write.len == 2) {
				uint16_t descr_value = param->write.value[1] << 8
						| param->write.value[0];
				if (descr_value == 0x0001) {
					if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
						ESP_LOGI(GATTS_TAG, "notify enable");
						uint8_t notify_data[15];
						for (int i = 0; i < sizeof(notify_data); ++i) {
							notify_data[i] = i % 0xff;
						}
						//the size of notify_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if,
								param->write.conn_id,
								gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								sizeof(notify_data), notify_data, false);
					}
				} else if (descr_value == 0x0002) {
					if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE) {
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i) {
							indicate_data[i] = i % 0xff;
						}
						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if,
								param->write.conn_id,
								gl_profile_tab[PROFILE_A_APP_ID].char_handle,
								sizeof(indicate_data), indicate_data, true);
					}
				} else if (descr_value == 0x0000) {
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
				} else {
					ESP_LOGE(GATTS_TAG, "unknown descr value");
					esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
				}

			}
		}
		example_write_event_env(gatts_if, &a_prepare_write_env, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT")
		;
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
				param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&a_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu)
		;
		ESP_LOGI(GATTS_TAG, "GATT server access interface : %d", gatts_if)
		;
		ESP_LOGI(GATTS_TAG, "connection id to indicate : %d",
				param->write.conn_id)
		;
		ESP_LOGI(GATTS_TAG, "attribute handle to indicate : %d",
				gl_profile_tab[PROFILE_A_APP_ID].char_handle)
		;
		break;
	case ESP_GATTS_CONF_EVT:
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG,
				"CREATE_SERVICE_EVT, status %d,  service_handle %d\n",
				param->create.status, param->create.service_handle)
		;
		gl_profile_tab[PROFILE_A_APP_ID].service_handle =
				param->create.service_handle;
		gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 =
		GATTS_CHAR_UUID_TEST_A;

		esp_ble_gatts_start_service(
				gl_profile_tab[PROFILE_A_APP_ID].service_handle);
		a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE
				| ESP_GATT_CHAR_PROP_BIT_NOTIFY;
		esp_err_t add_char_ret = esp_ble_gatts_add_char(
				gl_profile_tab[PROFILE_A_APP_ID].service_handle,
				&gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, a_property,
				&gatts_demo_char1_val, NULL);
		if (add_char_ret) {
			ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
		}
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT: {
		uint16_t length = 0;
		const uint8_t *prf_char;

		ESP_LOGI(GATTS_TAG,
				"ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle,
				param->add_char.service_handle);
		gl_profile_tab[PROFILE_A_APP_ID].char_handle =
				param->add_char.attr_handle;
		gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 =
		ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(
				param->add_char.attr_handle, &length, &prf_char);
		if (get_attr_ret == ESP_FAIL) {
			ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
		}

		ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x\n", length);
		for (int i = 0; i < length; i++) {
			ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n", i, prf_char[i]);
		}
		esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
				gl_profile_tab[PROFILE_A_APP_ID].service_handle,
				&gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
		if (add_descr_ret) {
			ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x",
					add_descr_ret);
		}
		connection.character_attr =
				gl_profile_tab[PROFILE_A_APP_ID].char_handle;
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[PROFILE_A_APP_ID].descr_handle =
				param->add_char.attr_handle;
		ESP_LOGI(GATTS_TAG,
				"ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle,
				param->add_char.service_handle)
		;
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle)
		;
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT: {
		esp_ble_conn_update_params_t conn_params = { 0 };
		memcpy(conn_params.bda, param->connect.remote_bda,
				sizeof(esp_bd_addr_t));
		/* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
		conn_params.latency = 0;
		conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
		conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
		conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
		ESP_LOGI(GATTS_TAG,
				"ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d",
				param->connect.conn_id, param->connect.remote_bda[0],
				param->connect.remote_bda[1], param->connect.remote_bda[2],
				param->connect.remote_bda[3], param->connect.remote_bda[4],
				param->connect.remote_bda[5], param->connect.is_connected);
		gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
		//start sent the update connection parameters to the peer device.
		esp_ble_gap_update_conn_params(&conn_params);

		printf(
				"***************** \n ***************** \n ***************** \n		CONNECTION	A	\n ***************** \n ***************** \n ***************** \n ");

		break;
	}
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT")
		;
		esp_ble_gap_start_advertising(&adv_params);
		break;
	case ESP_GATTS_OPEN_EVT:
	case ESP_GATTS_CANCEL_OPEN_EVT:
	case ESP_GATTS_CLOSE_EVT:
	case ESP_GATTS_LISTEN_EVT:
	case ESP_GATTS_CONGEST_EVT:
	default:
		break;
	}
}

static void gatts_profile_b_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	switch (event) {
	case ESP_GATTS_REG_EVT:
		ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n",
				param->reg.status, param->reg.app_id)
		;
		gl_profile_tab[PROFILE_B_APP_ID].service_id.is_primary = true;
		gl_profile_tab[PROFILE_B_APP_ID].service_id.id.inst_id = 0x00;
		gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.len =
		ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_B_APP_ID].service_id.id.uuid.uuid.uuid16 =
		GATTS_SERVICE_UUID_TEST_B;

		esp_ble_gatts_create_service(gatts_if,
				&gl_profile_tab[PROFILE_B_APP_ID].service_id,
				GATTS_NUM_HANDLE_TEST_B);
		break;
	case ESP_GATTS_READ_EVT: {
		ESP_LOGI(GATTS_TAG,
				"GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n",
				param->read.conn_id, param->read.trans_id, param->read.handle);
		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
		rsp.attr_value.handle = param->read.handle;
		rsp.attr_value.len = 4;
		rsp.attr_value.value[0] = 0xde;
		rsp.attr_value.value[1] = 0xed;
		rsp.attr_value.value[2] = 0xbe;
		rsp.attr_value.value[3] = 0xef;
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
				param->read.trans_id, ESP_GATT_OK, &rsp);
		break;
	}
	case ESP_GATTS_WRITE_EVT: {
		ESP_LOGI(GATTS_TAG,
				"GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n",
				param->write.conn_id, param->write.trans_id,
				param->write.handle);
		if (!param->write.is_prep) {
			ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :",
					param->write.len);
			esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
			if (gl_profile_tab[PROFILE_B_APP_ID].descr_handle
					== param->write.handle && param->write.len == 2) {
				uint16_t descr_value = param->write.value[1] << 8
						| param->write.value[0];
				if (descr_value == 0x0001) {
					if (b_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
						ESP_LOGI(GATTS_TAG, "notify enable");
						uint8_t notify_data[15];
						for (int i = 0; i < sizeof(notify_data); ++i) {
							notify_data[i] = i % 0xff;
						}
						//the size of notify_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if,
								param->write.conn_id,
								gl_profile_tab[PROFILE_B_APP_ID].char_handle,
								sizeof(notify_data), notify_data, false);
					}
				} else if (descr_value == 0x0002) {
					if (b_property & ESP_GATT_CHAR_PROP_BIT_INDICATE) {
						ESP_LOGI(GATTS_TAG, "indicate enable");
						uint8_t indicate_data[15];
						for (int i = 0; i < sizeof(indicate_data); ++i) {
							indicate_data[i] = i % 0xaa;
						}
						//the size of indicate_data[] need less than MTU size
						esp_ble_gatts_send_indicate(gatts_if,
								param->write.conn_id,
								gl_profile_tab[PROFILE_B_APP_ID].char_handle,
								sizeof(indicate_data), indicate_data, true);
					}
				} else if (descr_value == 0x0000) {
					ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
				} else {
					ESP_LOGE(GATTS_TAG, "unknown value");
				}

			}
		}
		example_write_event_env(gatts_if, &b_prepare_write_env, param);
		break;
	}
	case ESP_GATTS_EXEC_WRITE_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT")
		;
		esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
				param->write.trans_id, ESP_GATT_OK, NULL);
		example_exec_write_event_env(&b_prepare_write_env, param);
		break;
	case ESP_GATTS_MTU_EVT:
		ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu)
		;
		break;
	case ESP_GATTS_CONF_EVT:
	case ESP_GATTS_UNREG_EVT:
		break;
	case ESP_GATTS_CREATE_EVT:
		ESP_LOGI(GATTS_TAG,
				"CREATE_SERVICE_EVT, status %d,  service_handle %d\n",
				param->create.status, param->create.service_handle)
		;
		gl_profile_tab[PROFILE_B_APP_ID].service_handle =
				param->create.service_handle;
		gl_profile_tab[PROFILE_B_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_B_APP_ID].char_uuid.uuid.uuid16 =
		GATTS_CHAR_UUID_TEST_B;

		esp_ble_gatts_start_service(
				gl_profile_tab[PROFILE_B_APP_ID].service_handle);
		b_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE
				| ESP_GATT_CHAR_PROP_BIT_NOTIFY;
		esp_err_t add_char_ret = esp_ble_gatts_add_char(
				gl_profile_tab[PROFILE_B_APP_ID].service_handle,
				&gl_profile_tab[PROFILE_B_APP_ID].char_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, b_property,
				NULL, NULL);
		if (add_char_ret) {
			ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
		}
		break;
	case ESP_GATTS_ADD_INCL_SRVC_EVT:
		break;
	case ESP_GATTS_ADD_CHAR_EVT:
		ESP_LOGI(GATTS_TAG,
				"ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle,
				param->add_char.service_handle)
		;

		gl_profile_tab[PROFILE_B_APP_ID].char_handle =
				param->add_char.attr_handle;
		gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
		gl_profile_tab[PROFILE_B_APP_ID].descr_uuid.uuid.uuid16 =
		ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
		esp_ble_gatts_add_char_descr(
				gl_profile_tab[PROFILE_B_APP_ID].service_handle,
				&gl_profile_tab[PROFILE_B_APP_ID].descr_uuid,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				NULL, NULL);
		break;
	case ESP_GATTS_ADD_CHAR_DESCR_EVT:
		gl_profile_tab[PROFILE_B_APP_ID].descr_handle =
				param->add_char.attr_handle;
		ESP_LOGI(GATTS_TAG,
				"ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
				param->add_char.status, param->add_char.attr_handle,
				param->add_char.service_handle)
		;
		break;
	case ESP_GATTS_DELETE_EVT:
		break;
	case ESP_GATTS_START_EVT:
		ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
				param->start.status, param->start.service_handle)
		;
		break;
	case ESP_GATTS_STOP_EVT:
		break;
	case ESP_GATTS_CONNECT_EVT:
		ESP_LOGI(GATTS_TAG,
				"CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d",
				param->connect.conn_id, param->connect.remote_bda[0],
				param->connect.remote_bda[1], param->connect.remote_bda[2],
				param->connect.remote_bda[3], param->connect.remote_bda[4],
				param->connect.remote_bda[5], param->connect.is_connected)
		;
		gl_profile_tab[PROFILE_B_APP_ID].conn_id = param->connect.conn_id;

		printf(
						"***************** \n ***************** \n ***************** \n		CONNECTION	B	\n ***************** \n ***************** \n ***************** \n ");
		break;
	case ESP_GATTS_DISCONNECT_EVT:
	case ESP_GATTS_OPEN_EVT:
	case ESP_GATTS_CANCEL_OPEN_EVT:
	case ESP_GATTS_CLOSE_EVT:
	case ESP_GATTS_LISTEN_EVT:
	case ESP_GATTS_CONGEST_EVT:
	default:
		break;
	}
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
		esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	/* If event is register event, store the gatts_if for each profile */
	if (event == ESP_GATTS_REG_EVT) {
		if (param->reg.status == ESP_GATT_OK) {
			gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
		} else {
			ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
					param->reg.app_id, param->reg.status);
			return;
		}
	}

	/* If the gatts_if equal to profile A, call profile A cb handler,
	 * so here call each profile's callback */
	do {
		int idx;
		for (idx = 0; idx < PROFILE_NUM; idx++) {
			if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
			gatts_if == gl_profile_tab[idx].gatts_if) {
				if (gl_profile_tab[idx].gatts_cb) {
					gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
				}
			}
		}
	} while (0);
}

static void example_gpio_init() {

	gpio_config_t io_conf;
//disable interrupt
	io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_SEL_18;
//disable pull-down mode
	io_conf.pull_down_en = 0;
//disable pull-up mode
	io_conf.pull_up_en = 0;
//configure GPIO with the given settings
	gpio_config(&io_conf);

//interrupt of any edge
	io_conf.intr_type = GPIO_PIN_INTR_ANYEDGE;
//bit mask of the pins, use GPIO4/5 here
	io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
//enable pull-up mode
	io_conf.pull_up_en = 1;
	gpio_config(&io_conf);

	gpio_set_intr_type(GPIO_INPUT_IO_1, GPIO_INTR_POSEDGE);

//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
//hook isr handler for specific gpio pin
	gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler,
			(void*) GPIO_INPUT_IO_0);
	gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler,
			(void*) GPIO_INPUT_IO_1);

}

static void example_tg0_timer0_init() {
	int timer_group = TIMER_GROUP_0;
	int timer_idx = TIMER_0;
	timer_config_t config;
	config.alarm_en = 1;
	config.auto_reload = 1;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_SEL;
	config.counter_en = TIMER_PAUSE;
	/*Configure timer*/
	timer_init(timer_group, timer_idx, &config);
	/*Stop timer counter*/
	timer_pause(timer_group, timer_idx);
	/*Load counter value */
	timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
	/*Set alarm value*/
	timer_set_alarm_value(timer_group, timer_idx,
	TIMER_INTERVAL0_SEC * TIMER_SCALE);
	/*Enable timer interrupt*/
	timer_enable_intr(timer_group, timer_idx);
	/*Set ISR handler*/
	timer_isr_register(timer_group, timer_idx, timer_group0_isr,
			(void*) timer_idx,
			ESP_INTR_FLAG_IRAM, NULL);
	/*Start timer counter*/
	/*timer_start(timer_group, timer_idx);*/
}

static void example_tg0_timer1_init() {
	int timer_group = TIMER_GROUP_0;
	int timer_idx = TIMER_1;
	timer_config_t config;
	config.alarm_en = 1;
	config.auto_reload = 1;
	config.counter_dir = TIMER_COUNT_UP;
	config.divider = TIMER_DIVIDER;
	config.intr_type = TIMER_INTR_SEL;
	config.counter_en = TIMER_PAUSE;
	/*Configure timer*/
	timer_init(timer_group, timer_idx, &config);
	/*Stop timer counter*/
	timer_pause(timer_group, timer_idx);
	/*Load counter value */
	timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
	/*Set alarm value*/
	timer_set_alarm_value(timer_group, timer_idx,
	TIMER_INTERVAL1_SEC * TIMER_SCALE);
	/*Enable timer interrupt*/
	timer_enable_intr(timer_group, timer_idx);
	/*Set ISR handler*/
	timer_isr_register(timer_group, timer_idx, timer_group0_isr,
			(void*) timer_idx,
			ESP_INTR_FLAG_IRAM, NULL);
	/*Start timer counter*/
	/*timer_start(timer_group, timer_idx);*/
}

void notify_data(esp_gatt_if_t gatt_server_if, uint16_t attr_handle,
		uint8_t *value, uint8_t length) {
	/* notify_data(connection.gatt_if, connection.character_attr, notify_tab, sizeof(notify_tab));*/

	if (gatt_server_if == ESP_GATT_IF_NONE) {

		ESP_LOGE(GATTS_TAG, "Cannot send indicate - gatt_if is NONE");

	} else {

		esp_gatt_rsp_t rsp;
		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

		rsp.attr_value.len = 4;
		rsp.attr_value.value[0] = *value;
		value++;
		rsp.attr_value.value[1] = *value;
		value++;
		rsp.attr_value.value[2] = *value;
		value++;
		rsp.attr_value.value[3] = *value;

		esp_ble_gatts_send_indicate(gatt_server_if, 0, attr_handle,
				rsp.attr_value.len, rsp.attr_value.value, false);

	}

}

// ----------------------------------------- FUNCTIONS ---------------------------------------------------------------------------------- */

// ----------------------------------------- MAIN --------------------------------------------------------------------------------------- */

void app_main() {
	esp_err_t ret;

	/* Initialize NVS. */
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()
	;
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s initialize controller failed\n", __func__);
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s enable controller failed\n", __func__);
		return;
	}
	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s init bluetooth failed\n", __func__);
		return;
	}
	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed\n", __func__);
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}
	ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
	if (ret) {
		ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
		return;
	}
	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret) {
		ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x",
				local_mtu_ret);
	}

	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	timer_queue = xQueueCreate(10, sizeof(timer_event_t));

	example_gpio_init();
	example_tg0_timer0_init();
	example_tg0_timer1_init();

	xTaskCreate(vibro_task, "vibro_task", 4096, NULL, 10, NULL);
	xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

	int cnt = 0;
	while (1) {
		printf("cnt: %d\n", cnt++);
		vTaskDelay(1000 / portTICK_RATE_MS);
		gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
	}

	return;
}
