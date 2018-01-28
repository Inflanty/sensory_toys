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
#define ESP_SERVER_NUMBER_A 0x01
#define ESP_SERVER_NUMBER_B 0x02
#define ESP_SERVER_NUMBER_C 0x03
#define ESP_SERVER_NUMBER_D 0x04
#define UART1_tx 17
#define UART1_rx 16
#define CONNECTING 0xAA
#define ALL_CONNECTED 0xFF
#define CONNECTION_TIMEOUT 0xEE

#define MASTER_COMMAND	0xFF
#define STBY 			0xAA
#define INACTION_TIME   10
// ----------------------------------------- USER DEFINE---------------------------------------------------------------------------------- */

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
conn_params connection_a, connection_b, connection_c, connection_d;
static QueueHandle_t uart0_queue;
static QueueHandle_t player_queue;

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


/* INTO MAIN */
uart_init();

player_queue = xQueueCreate(10, sizeof(uint32_t));
xTaskCreate(player_task, "player_task", 2048, NULL, 10, NULL);

/*while (conn_device_a == false){
 vTaskDelay (500 / portTICK_PERIOD_MS);
 printf ("\nNO DEVICE");
 };*/
do {
  if ((last_message == true) && (conn_device_a == true) && (conn_device_b == true) && (conn_device_c == true) && (conn_device_d == true)) {
    //printf("\nlast message");
    gettimeofday(&now, NULL);
    if ((now.tv_sec - receive.tv_sec) > INACTION_TIME) {
      uint8_t write_char_data[] = { MASTER_COMMAND, 0x0F, 0xF0, STBY };

      /* STANDBY REQUEST TO DEVICE A */
      esp_ble_gattc_write_char(connection_a.gatt_if,
          gl_profile_tab[PROFILE_A_APP_ID].conn_id,
          gl_profile_tab[PROFILE_A_APP_ID].char_handle,
          sizeof(write_char_data), write_char_data,
          ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      printf("\nSEND STANDBY REQUEST TO DEVICE A");

      /* STANDBY REQUEST TO DEVICE B */
      esp_ble_gattc_write_char(connection_b.gatt_if,
          gl_profile_tab[PROFILE_B_APP_ID].conn_id,
          gl_profile_tab[PROFILE_B_APP_ID].char_handle,
          sizeof(write_char_data), write_char_data,
          ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      printf("\nSEND STANDBY REQUEST TO DEVICE B");

      /* STANDBY REQUEST TO DEVICE C */
      esp_ble_gattc_write_char(connection_c.gatt_if,
          gl_profile_tab[PROFILE_C_APP_ID].conn_id,
          gl_profile_tab[PROFILE_C_APP_ID].char_handle,
          sizeof(write_char_data), write_char_data,
          ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      printf("\nSEND STANDBY REQUEST TO DEVICE C");

      /* STANDBY REQUEST TO DEVICE B */
      esp_ble_gattc_write_char(connection_d.gatt_if,
          gl_profile_tab[PROFILE_D_APP_ID].conn_id,
          gl_profile_tab[PROFILE_D_APP_ID].char_handle,
          sizeof(write_char_data), write_char_data,
          ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      printf("\nSEND STANDBY REQUEST TO DEVICE D");
      last_message = false;
    }
  }
} while (1);
