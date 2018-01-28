// ---------------------------------------- USER DEFINE --------------------------------------------------------------------------------- */
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

#define EX_UART_NUM UART_NUM_0

#define BUF_SIZE (1024)

#define EXT_WORK 0x10
#define EXT_ENERGY 0x20

#define WATER 	0x30
#define AIR 	0x40
#define WOOD	0x50
#define STONE	0x60

#define MASTER_COMMAND	0xFF
#define STBY 			0xAA
#define INACTION_TIME   10
// ---------------------------------------- USER DEFINE --------------------------------------------------------------------------------- */

// ----------------------------------------- USER DATA ---------------------------------------------------------------------------------- */
static const char *TAG = "uart_events";

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

struct timeval stby_counting, now;

bool inaction = false;
//
// ----------------------------------------- USER DATA ---------------------------------------------------------------------------------- */

// ----------------------------------------- USER FNC ---------------------------------------------------------------------------------- */
static QueueHandle_t uart0_queue;
static xQueueHandle gpio_evt_queue = NULL;
xQueueHandle timer_queue;

static void exp_task(void *pvParameters) {
	uart_event_t event;
	size_t buffered_size;
	uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);

	for (;;) {
		//Waiting for UART event.
		if (xQueueReceive(uart0_queue, (void * )&event,
				(portTickType)portMAX_DELAY)) {
			ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
			switch (event.type) {
			//Event of UART receving data
			/*We'd better handler data event fast, there would be much more data events than
			 other types of events. If we take too much time on data event, the queue might
			 be full.
			 in this example, we don't process data in event, but read data outside.*/
			case UART_DATA:
				uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
				ESP_LOGI(TAG, "data, len: %d; buffered len: %d", event.size,
						buffered_size)
				;

				int len = uart_read_bytes(EX_UART_NUM, dtmp, BUF_SIZE,
						100 / portTICK_RATE_MS);
				if (len == 1) {
					if (dtmp[0] != MY_ELEMENT) {
						module.evt = true;
						module.level = dtmp[0];
						xuTimerStart(TIMER_GROUP_0, TIMER_1, 0, 0);
					} else if (dtmp[0] == MY_ELEMENT) {
						module.level = EXP_DISCONNECT;
						module.evt = false;
					}

				} else if (len != 1) {
					module.level = EXP_DISCONNECT;
					module.evt = false;
				}
				break;
				//Event of HW FIFO overflow detected
			case UART_FIFO_OVF:
				ESP_LOGI(TAG, "hw fifo overflow\n")
				;
				//If fifo overflow happened, you should consider adding flow control for your application.
				//We can read data out out the buffer, or directly flush the rx buffer.
				uart_flush(EX_UART_NUM);
				break;
				//Event of UART ring buffer full
			case UART_BUFFER_FULL:
				ESP_LOGI(TAG, "ring buffer full\n")
				;
				//If buffer full happened, you should consider encreasing your buffer size
				//We can read data out out the buffer, or directly flush the rx buffer.
				uart_flush(EX_UART_NUM);
				break;
				//Event of UART RX break detected
			case UART_BREAK:
				ESP_LOGI(TAG, "uart rx break\n")
				;
				break;
				//Event of UART parity check error
			case UART_PARITY_ERR:
				ESP_LOGI(TAG, "uart parity error\n")
				;
				break;
				//Event of UART frame error
			case UART_FRAME_ERR:
				ESP_LOGI(TAG, "uart frame error\n")
				;
				break;
				//UART_PATTERN_DET
			case UART_PATTERN_DET:
				ESP_LOGI(TAG, "uart pattern detected\n")
				;
				break;
				//Others
			default:
				ESP_LOGI(TAG, "uart event type: %d\n", event.type)
				;
				break;
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);


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
  				module.state = MOTION;
  				xuTimerStart(TIMER_GROUP_0, TIMER_1, 0, 0);
  			} else if (io_num == GPIO_INPUT_IO_1) {
  				/* mozna ustawic polaczenie z rozszezeniem */
  			}
  		}

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
  			module.level = EXP_DISCONNECT;
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

  void xuNotifyData(esp_gatt_if_t gatt_server_if, uint16_t attr_handle,
  		uint8_t *value, uint8_t length) {
  	/* notify_data(connection.gatt_if, connection.character_attr, notify_tab, sizeof(notify_tab));*/

  	if (gatt_server_if == ESP_GATT_IF_NONE) {

  		ESP_LOGE(GATTS_TAG, "Cannot send indicate - gatt_if is NONE");

  	} else {

  		esp_gatt_rsp_t rsp;
  		memset(&rsp, 0, sizeof(esp_gatt_rsp_t));

  		uint8_t notify_tab[length];
  		notify_tab[0] = *value;
  		value++;
  		notify_tab[1] = *value;
  		value++;
  		notify_tab[2] = *value;
  		value++;
  		notify_tab[3] = *value;

  		printf("SENDING NOTIFICANTION\n");

  		//the size of notify_data[] need less than MTU size
  		esp_ble_gatts_send_indicate(connection.gatt_if, 0,
  				connection.character_attr, length, notify_tab,
  				false);
  	}
  }

  void xuTimerStart(timer_group_t group_num, timer_idx_t timer_num,
  		uint64_t load_val, uint64_t counter_val) {
  	timer_get_counter_value(group_num, timer_num, &counter_val);
  	if (counter_val != 0x00000000ULL) {
  		timer_pause(group_num, timer_num);
  		if (timer_set_counter_value(group_num, timer_num, load_val) == ESP_OK) {
  			if (timer_start(group_num, timer_num) == ESP_OK) {
  				printf("Timer_start find motion\n");
  				if (TIMERG0.hw_timer[timer_num].config.alarm_en == 1) {

  				} else {
  					TIMERG0.hw_timer[timer_num].config.alarm_en = 1;
  				}
  			}
  		}
  	} else if (timer_start(group_num, timer_num) == ESP_OK) {
  		printf("Timer_start find motion\n");
  		if (TIMERG0.hw_timer[timer_num].config.alarm_en == 1) {

  		} else {
  			TIMERG0.hw_timer[timer_num].config.alarm_en = 1;
  		}
  	}
  }

  void xvStbyAction() {
  	uint8_t confirm[4] = { 0 };
  	confirm[0] = MASTER_COMMAND;
  	confirm[1] = 0x00;
  	confirm[2] = 0x00;
  	confirm[4] = STBY;
  	xuNotifyData(connection.gatt_if, connection.character_attr, confirm, 4);
  	vTaskDelay(5000 / portTICK_PERIOD_MS);
  	/* STANDBY MODE ON */
  	const int ext_wakeup_pin_1 = 4;
  	const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

  	printf("Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
  	int lev = rtc_gpio_get_level(4);
  	if (lev == 1) {
  		esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask,
  				ESP_EXT1_WAKEUP_ALL_LOW);
  	} else {
  		esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask,
  				ESP_EXT1_WAKEUP_ANY_HIGH);
  	}
  	printf("STARTING DEEP SLEEP\nWAKEUP PIN LEVEL : %d\n", lev);

  	if (esp_bluedroid_disable() == ESP_OK) {
  		vTaskDelay(100 / portTICK_PERIOD_MS);
  		printf("BLUEDROID DISABLE\n");
  		if (esp_bt_controller_disable() == ESP_OK) {
  			printf("BT CONTROLLER DISABLE\n");
  			printf("DEEP SLEEP START NOW\n");
  			vTaskDelay(100 / portTICK_PERIOD_MS);
  			esp_deep_sleep_start();
  		}
  	}

    static void uart_init() {
    	uart_config_t uart_config = { .baud_rate = 115200, .data_bits =
    			UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
    			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    			.rx_flow_ctrl_thresh = 122, };
    //Set UART parameters
    	uart_param_config(EX_UART_NUM, &uart_config);
    //Set UART log level
    	esp_log_level_set(TAG, ESP_LOG_INFO);
    //Install UART driver, and get the queue.
    	uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 10,
    			&uart0_queue, 0);

    //Set UART pins (using UART0 default pins ie no changes.)
    	uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
    	UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    	uart_enable_pattern_det_intr(EX_UART_NUM, '+', 3, 10000, 10, 10);
    //Create a task to handler UART event from ISR
    	xTaskCreate(exp_task, "exp_task", 2048, NULL, 12, NULL);
    }

    /* INTO MAIN, AFTER SYSTEM INICIALIZATION */
    vTaskDelay (5000 / portTICK_PERIOD_MS);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    example_gpio_init();
    example_tg0_timer0_init();
    example_tg0_timer1_init();
    uart_init();

    xTaskCreate(vibro_task, "vibro_task", 4096, NULL, 10, NULL);
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    while (1) {
      if (inaction == true) {
        gettimeofday(&now, NULL);
        if ((now.tv_sec - stby_counting.tv_sec) > (INACTION_TIME)) {
          inaction = false;
          xvStbyAction();
        }
      }
      vTaskDelay (18 / portTICK_PERIOD_MS);
    }
