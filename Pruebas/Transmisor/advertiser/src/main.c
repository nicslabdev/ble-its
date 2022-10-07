#include <zephyr.h>
#include <drivers/uart.h>
#include <device.h>
#include <soc.h>
#include <sys/printk.h>
#include <string.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <sys/util.h>
#include <errno.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/uuid.h>
#include <settings/settings.h>
#include <stdio.h>
#include <dk_buttons_and_leds.h>
#include <logging/log.h>
#include <logging/log_core.h>
#include <logging/log_ctrl.h>
#include <mpsl_radio_notification.h>
#include <mpsl.h>
#include <time.h>
#include <kernel.h>


// LOG CONFIG PARAMETERS
// ------------------------------------------------
#define LOG_MODULE_NAME its_ble_tx_poc
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);
// ------------------------------------------------


//BLE CONFIG PARAMETERS
// ------------------------------------------------
#define DEVICE_NAME_ CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define DEVICE_NAME_TIME_SYNC "SYN"
#define DEVICE_NAME_TIME_SYNC_LEN (sizeof(DEVICE_NAME_TIME_SYNC) - 1)

#define TAMA_BUF 238 //tamaño máximo de datos sin que haya fragmentación
#define TAMA_TIME 8 //tamaño del timestamp

uint8_t its_buf[TAMA_BUF+2]; //buffer con la trama ITS
uint8_t time_buf[TAMA_TIME + 2]; //buffer con la información temporal

struct bt_data ad_its[3] = { 
	//BT_DATA_BYTES(type of advertising data field, single-byte parameters)
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR|BT_LE_AD_GENERAL), //BR/EDR not supported, general discoverable (no timeout)
	//BT_DATA_BYTES(BT_DATA_UUID128_ALL, CUSTOM_SERVICE_UUID),
	BT_DATA_BYTES(BT_DATA_NAME_SHORTENED, DEVICE_NAME_),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, its_buf, ARRAY_SIZE(its_buf)) 
};
struct bt_data ad_time[3] = { 
	//BT_DATA_BYTES(type of advertising data field, single-byte parameters)
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR|BT_LE_AD_GENERAL), //BR/EDR not supported, general discoverable (no timeout)
	//BT_DATA_BYTES(BT_DATA_UUID128_ALL, CUSTOM_SERVICE_UUID),
	BT_DATA_BYTES(BT_DATA_NAME_SHORTENED, DEVICE_NAME_TIME_SYNC),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, time_buf, ARRAY_SIZE(time_buf)) 
};

struct bt_le_ext_adv *adv; 
// ------------------------------------------------

// ITS FRAMES
// ------------------------------------------------
//Trama sin seguridad (tamaño 54 bytes, 55 con el tamaño de la trama ITS)
uint8_t non_secure_frame[] = { 0x36, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x89, 0x47, 0x11, 0x00, 0x1a, 0x01, 0x20, 0x50, 0x00, 0x80, 0x00, 0x54, 0x01, 0x00, 0x80, 0x00,
							   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x2c, 0xda, 0x01, 0x1d, 0x11, 0x3b, 0x88, 0x06, 0xd0, 0x65, 0x27, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


//Trama con hash (tamaño 211 bytes, 212 con el tamaño de la trama ITS)
uint8_t hash_frame[] = {	0xd3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x89, 0x47, 0x12, 0x00, 0x1a, 0x01, 0x03, 0x15, 0x80, 0x01, 0xaf, 0x6d,
							0x21, 0x7a, 0x1e, 0xb3, 0x24, 0x9d, 0x00, 0x00, 0x02, 0x15, 0x76, 0x09, 0xa5, 0xe6, 0x1b, 0x05, 0x24, 0x01, 0x64, 0x20, 0x50, 0x00, 0x80, 0x00, 0x40,
							0x01, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0xe7, 0xd5, 0x08, 0x1d, 0x11, 0x3b, 0x88, 0x06, 0xd0, 0x65, 0x27, 0x80, 0x00, 0x00,
							0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xd1, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x01, 0xd8, 0x8c, 0x00, 0x5a, 0x56, 0xc4, 0x91, 0x8e, 0x43, 0x46,
							0xe5, 0x1f, 0xff, 0xff, 0xfc, 0x23, 0xb7, 0x74, 0x3e, 0x7f, 0x00, 0x01, 0x20, 0x00, 0x00, 0x3f, 0xe1, 0xed, 0x04, 0x03, 0xff, 0xe3, 0xff, 0xf4, 0x01,
							0x07, 0xff, 0xfa, 0x83, 0x9a, 0x83, 0x98, 0x3a, 0xd2, 0x74, 0x80, 0x75, 0xa4, 0xe9, 0x00, 0x40, 0x00, 0x00, 0x00, 0x43, 0x01, 0x00, 0x00, 0x6a, 0x52,
							0x4e, 0x4e, 0xc2, 0x70, 0x8a, 0x2e, 0x73, 0x62, 0xd1, 0x49, 0x65, 0xea, 0xc2, 0xb2, 0x70, 0xba, 0x3f, 0x6e, 0x8f, 0xbf, 0x4a, 0xc8, 0xde, 0x32, 0xd9,
							0xde, 0xcc, 0x10, 0x5f, 0x5e, 0x5e, 0x4c, 0xbd, 0x99, 0xf6, 0x35, 0x87, 0x68, 0xea, 0x3b, 0xbb, 0x64, 0xeb, 0x37, 0xcf, 0x0a, 0x42, 0x35, 0xa4, 0xd8,
							0xe5, 0x3f, 0x20, 0x35, 0x23, 0xed, 0xe5, 0x04, 0x7f, 0x60, 0x25, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x00, 0x00	};


// Trama certificado (tamaño 199 bytes, 200 con el tamaño de la trama ITS)
uint8_t certif_frame[] = {	0xc7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x89, 0x47, 0x12, 0x00, 0x1a, 0x01, 0x03, 0x80, 0xaf, 0x80, 0x02, 0x02,
							0x01, 0xc8, 0x97, 0x1d, 0x82, 0x47, 0x73, 0xdb, 0xe5, 0x01, 0x00, 0x52, 0x00, 0x00, 0x04, 0x4b, 0xfb, 0x65, 0x53, 0x28, 0x0b, 0x5c, 0xaa, 0x48,
							0x14, 0xb0, 0x79, 0x2a, 0xbb, 0x2f, 0x23, 0x73, 0xe3, 0x01, 0x33, 0xd8, 0xb4, 0x0c, 0xeb, 0xb6, 0x57, 0x59, 0x43, 0x16, 0x15, 0x27, 0xd5, 0x86,
							0x09, 0x8f, 0x74, 0xa0, 0x90, 0x24, 0x26, 0x2f, 0x26, 0x50, 0x47, 0x6f, 0x5b, 0x91, 0x74, 0x53, 0x7f, 0x61, 0xc2, 0x48, 0xc3, 0x84, 0xb2, 0xf7,
							0xe0, 0xe5, 0x6c, 0x97, 0x97, 0x28, 0xf4, 0x02, 0x00, 0x21, 0x0b, 0x24, 0x03, 0x01, 0x00, 0x00, 0x25, 0x04, 0x01, 0x00, 0x00, 0x00, 0x09, 0x01,
							0x22, 0xc9, 0xf6, 0x30, 0x22, 0xd3, 0x3e, 0xc0, 0x00, 0x00, 0xfb, 0x9e, 0xd4, 0x28, 0x43, 0x8f, 0xd6, 0xb8, 0x1f, 0xaa, 0xb6, 0x00, 0x4d, 0x2a,
							0x02, 0x26, 0x26, 0x59, 0x57, 0xd9, 0xe0, 0x6b, 0xcd, 0x4b, 0xa4, 0x90, 0x4d, 0xda, 0x9d, 0xf3, 0x18, 0x1a, 0x76, 0x85, 0xba, 0x23, 0x0b, 0xd5,
							0xa1, 0x03, 0xde, 0x86, 0xe0, 0x35, 0x33, 0x5c, 0xed, 0x0c, 0x5a, 0x6e, 0xf8, 0x6f, 0x83, 0x05, 0x2f, 0xd5, 0x76, 0x97, 0xfd, 0xdb, 0x40, 0x36,
							0x70, 0xdc, 0x05, 0x24, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define DEVICE_ID 1
// ------------------------------------------------


// EVENTS
// ------------------------------------------------
K_EVENT_DEFINE(event);
// ------------------------------------------------

//SEPAMHORES
// ------------------------------------------------
K_SEM_DEFINE(time_sync_sem, 0, 1);
// ------------------------------------------------


// IMPLEMENTED FUNCTIONS
// ------------------------------------------------
static void uart1_irq_handler(const struct device *dev, void *context);
static void timer_callback(struct k_timer *timer);
void work_handler(struct k_work *work);
void update_frames_handler(struct k_work *work);
void update_time_handler(struct k_work *work);
void radio_handler(const void *context);
void hebra(void);
void timer_fin(struct k_timer *timer);
static void test_timer_callback(struct k_timer *timer); //prueba

// ------------------------------------------------

// TIMER
// ------------------------------------------------
#define PER 300 //ms
K_TIMER_DEFINE(timer, timer_callback, timer_fin);
K_WORK_DEFINE(my_work, work_handler);

#define BURST_TIME 30 * 1000 //ms
K_TIMER_DEFINE(test_timer, test_timer_callback, NULL); //prueba
// ------------------------------------------------

//STATES
// ------------------------------------------------
enum states {init, time_sync, tx};
// ------------------------------------------------


// COMANDOS
// ------------------------------------------------
//nrf9160 SiP -> nrf52840 SoC
enum commands_9160_5280_time_sync {finish_sync = 0x01};
enum commands_9160_5280 {scenario_1 = 0x01, scenario_2 = 0x02, scenario_3 = 0x04, 
						 test_started = 0x08, test_finished = 0x10, 
						 per_1 = 0x20, per_2 = 0x40};
//nrf52840 SoC -> nrf9160 SiP 
enum commands_5280_9160 {advert_ready = 0x80};
// ------------------------------------------------


// GLOBAL VARIABLES
// ------------------------------------------------
static const struct device *uart1;

bool send_certificate = true;
uint8_t scenario;
uint8_t state = init;
uint32_t int_min,int_max;
bool advertising_on = true; //prueba
uint16_t contador = 0; //número de tramas emitidas

K_WORK_DEFINE(update_its_work, update_frames_handler);
K_WORK_DEFINE(update_time_work, update_time_handler);

uint8_t info[20];
bool command = true;
int cont = 0;

// ------------------------------------------------

// THREAD CONFIG PARAMETERS
// ------------------------------------------------
/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7

K_THREAD_DEFINE(h, STACKSIZE, hebra, NULL, NULL, NULL, PRIORITY, 0, 0);
// ------------------------------------------------


void main(void){
	int32_t err;
	uint8_t buf[8]; //Se almacena el tiempo que se recibe a través de la uart
	int64_t unix_time_ns;
	int aux = 0;
	unsigned char c;
	struct timespec received_time;
	struct timespec time_now;

	memset(buf, '\0', sizeof(buf));
	memset(its_buf, '\0', sizeof(its_buf));
	memset(time_buf, '\0', sizeof(time_buf));

	//inicialización del logger
	log_init();
	LOG_INF("Starting ITS BLE (TX)");

	//Inicialización de la uart que conecta con el nrf9160 SiP
	uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));
	__ASSERT(uart1, "Failed to get the device");

	//Espera a recibir información temporal del nrf9160 SiP
	LOG_INF("Esperando a recibir a través de la uart");
	while(aux < 8){
		err = uart_poll_in(uart1, &c);
		if(err == 0){
			//Se ha recibido un caracter
			printk("%02x ", c);
			buf[aux] = c;
			aux++;
		}
	}

	memcpy(&unix_time_ns, buf, sizeof(buf));
	received_time.tv_sec = unix_time_ns / 1E9;
	received_time.tv_nsec = unix_time_ns % (int64_t)1E9;

	err = clock_settime(CLOCK_REALTIME, &received_time);
	if (err != 0) {
		LOG_ERR("(clock_settime) error %d", err);
		while(1);
	}
	LOG_INF("Obtenida la información temporal");
	LOG_INF("Unix_time_ns: %lld", unix_time_ns);


	//Radio notification
	err = mpsl_radio_notification_cfg_set(MPSL_RADIO_NOTIFICATION_TYPE_INT_ON_ACTIVE,MPSL_RADIO_NOTIFICATION_DISTANCE_200US,SWI1_IRQn);
	printk("mpsl_radio_notification (%d)", err);
	if (err < 0) {
		LOG_ERR("mpsl_radio_notification_cfg_set failed (err %d)\n", err);
		while(1);
	}

	IRQ_CONNECT(SWI1_IRQn, 5, radio_handler, NULL, 0);
	irq_enable(SWI1_IRQn);

	//Inicialización sistema Bluetooth
	err = bt_enable(NULL); 
	if (err) {
		LOG_ERR("bt_enable: error %d\n", err);
		while(1);
	}

	LOG_INF("Bluetooth initialized");

	//Configuración del advertisement extendido
	struct bt_le_adv_param* adv_param = 
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, //advertising non-connectable y non-scannable (esto ultimo quiere decir que no soporta un mensaje Scan Request)
		0x0320, //500 ms intervalo minimo
		0x0640, //1 s intervalo máximo
		NULL); //Dirección del peer: NULL porque el advertising es undirected

	err = settings_load();
	if(err < 0){
		LOG_ERR("(settings_load) error %d \n", err);
		while(1);
	}

	// Se crea el advertisement set
	err = bt_le_ext_adv_create(adv_param, NULL, &adv);
	if (err < 0) {
		LOG_ERR("(bt_le_ext_adv_create) Advertising set creation failed (err %d)\n", err);
		while(1);
	}

	//Periodo de sincronización temporal
	state = time_sync;

	time_buf[0] = 0x59;
	time_buf[1] = 0x00;
	memset(time_buf+2, '\0', TAMA_TIME); 

	err = clock_gettime(CLOCK_REALTIME, &time_now);
	if(err){
		LOG_ERR("clock_gettime error (%d)", err);
		while(1);
	}

	unix_time_ns = time_now.tv_sec * 1E9 + time_now.tv_nsec;
	LOG_INF("unix_time_ns: %lld", unix_time_ns);

	//llamar a clock_gettime directamente y luego pasar a nanosegundos
	//puede ser por date_time_now por la cache
	//activar un segundo el advertisement, paro, activo no se sabe cada cuanto se envia, 

	memcpy(time_buf+2, &unix_time_ns, sizeof(int64_t));

	//Se establecen los datos que se van a anunciar
	err = bt_le_ext_adv_set_data(adv, ad_time, ARRAY_SIZE(ad_time), NULL, 0);
	if (err < 0) {
		LOG_ERR("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err); 
		while(1);
	}

	LOG_INF("Inicio del periodo de sincronización");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err < 0)
	{
		LOG_ERR("(bt_le_ext_adv_start) Advertising start failed (err %d)\n", err);
		while (1);
	}

	//Establezco ISR de la UART
	uart_irq_callback_set(uart1, uart1_irq_handler);
	//Se habilitan las interrupciones
	uart_irq_rx_enable(uart1);

	//Esperar hasta que se pulse el botón: se pulsará cuando se ponga en hora
	k_sem_take(&time_sync_sem, K_FOREVER);

	LOG_INF("Finaliza el periodo de sincronización");
	err = bt_le_ext_adv_stop(adv);
	if(err < 0){
		LOG_ERR("(bt_le_ext_adv_stop) error %d \n", err);
		while(1);
	}

	state = tx;

	//Escenario e intervalo de advertisement inicial
	scenario = scenario_3;

	LOG_INF("Envío del comando advert_ready");

	//Indico al nrf9160 SiP que ya esta listo para transmitir (espero un tiempo porque si se envía directamente puede llegar a perderse)
	k_sleep(K_MSEC(2000));
	uart_poll_out(uart1, advert_ready);

	k_timer_start(&test_timer, K_MSEC(BURST_TIME), K_MSEC(BURST_TIME));

}


void update_frames_handler(struct k_work *work){
	int err;
	int64_t unix_time_ns;
	struct timespec time_now;

	LOG_INF("---------------");
	LOG_INF("Trama número %d", contador);
	memcpy(its_buf+2, &contador, sizeof(uint16_t));
	uint8_t aux = its_buf[2];
	its_buf[2] = its_buf[3];
	its_buf[3] = aux;
	contador++;

	err = clock_gettime(CLOCK_REALTIME, &time_now);
	if(err){
		LOG_ERR("clock_gettime error (%d)", err);
		while(1);
	}
	
	unix_time_ns = time_now.tv_sec * 1E9 + time_now.tv_nsec;
	LOG_INF("unix_time_ns: %lld", unix_time_ns);
	memcpy(its_buf+4, &unix_time_ns, sizeof(int64_t));

	//corregimos el efecto del memcpy, que altera el orden de los bytes
	/*for(int i = 0; i < 4; i++){
		aux = its_buf[i + 4];
		its_buf[i + 4] = its_buf[11 - i];
		its_buf[11 - i] = aux;
	}*/

	//Se actualiza el advertisement
	err = bt_le_ext_adv_set_data(adv, ad_its, ARRAY_SIZE(ad_its), NULL, 0);
	if (err < 0) {
		LOG_ERR("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err);
	}
	LOG_INF("---------------");

}


void update_time_handler(struct k_work *work){
	int err;
	uint8_t aux;
	struct timespec time_now;
	int64_t unix_time_ns;

	LOG_INF("---------------");
	err = clock_gettime(CLOCK_REALTIME, &time_now);
	if(err){
		LOG_ERR("clock_gettime error (%d)", err);
		while(1);
	}
	
	unix_time_ns = time_now.tv_sec * 1E9 + time_now.tv_nsec;
	LOG_INF("unix_time_ns: %lld", unix_time_ns);
	memcpy(time_buf+2, &unix_time_ns, sizeof(int64_t));

	//corregimos el efecto del memcpy, que altera el orden de los bytes
	/*for(int i = 0; i < 4; i++){
		aux = its_buf[i + 2];
		its_buf[i + 2] = its_buf[9 - i];
		its_buf[9 - i] = aux;
	}*/

	//Se actualiza el advertisement
	err = bt_le_ext_adv_set_data(adv, ad_time, ARRAY_SIZE(ad_time), NULL, 0);
	if (err < 0) {
		LOG_ERR("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err);
	}
	LOG_INF("---------------");

}


//Se ejecuta justo antes de que se vaya a encender la radio
void radio_handler(const void *context)
{
	//Al tratarse de un ISR la actualización del advertisement se deja a una hebra
	if(state == tx){
		k_work_submit(&update_its_work);
	}else{
		k_work_submit(&update_time_work);
	}
	
}


static void uart1_irq_handler(const struct device *dev, void *context){
	int err;
	uint8_t c;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			err = uart_fifo_read(dev, &c, sizeof(char)); 
			if(err <= 0){
				LOG_ERR("(uart_fifo_read) error %d", err);
			}else{
				LOG_INF("Recibido %02x a través de la uart", c);

				switch(state){
					case time_sync:
						if((c & finish_sync) != 0){
							k_sem_give(&time_sync_sem);
						}
					break;

					case tx:
						/*if((c & scenario_1) != 0){
							k_event_set(&event, scenario_1);
						}else if((c & scenario_2) != 0){
							k_event_set(&event, scenario_2);
						}else if((c & scenario_3) != 0){
							k_event_set(&event, scenario_3);
						}else if((c & test_started) != 0){
							k_event_set(&event, test_started);
						}else if((c & test_finished) != 0){
							k_event_set(&event, test_finished);
						}else if((c & per_1) != 0){
							k_event_set(&event, per_1);
						}else if((c & per_2) != 0){
							k_event_set(&event, per_2);
						}*/
					break;
				}

			}
		}
	}
}


//Cada vez que expira el timer se envía el certificado o el hash alternados
void work_handler(struct k_work *work)
{
    if(send_certificate){
		memcpy(its_buf+13, certif_frame, sizeof(certif_frame));
		send_certificate = false;
	}else{
		memcpy(its_buf+13, hash_frame, sizeof(hash_frame)); 
		send_certificate = true;
	}

	//No es necesario parar el advertisement porque no ocupa más de 251 bytes (no hay fragmentación)	
	int err = bt_le_ext_adv_set_data(adv, ad_its, ARRAY_SIZE(ad_its), NULL, 0);
	if (err < 0) {
		LOG_ERR("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err);
		while(1);
	}
}

//Se deja el procesado a otra hebra
static void timer_callback(struct k_timer *timer){
	k_work_submit(&my_work);
}


static void test_timer_callback(struct k_timer *timer){//prueba
	if(advertising_on){
		printk("advertising finished\n");
		advertising_on = false;
		k_event_set(&event, test_finished);
	}else{
		printk("advertising started\n");
		advertising_on = true;
		k_event_set(&event, test_started);
	}
} 


void timer_fin(struct k_timer *timer){
	LOG_INF("Timer_fin");
}


void hebra(void){
	uint32_t e;
	int err;
	struct bt_le_adv_param* adv_param;
	int64_t unix_time_ns;
	struct timespec time_now;

	LOG_INF("Se inicia la hebra");

	while(1){

		//La hebra se bloquea esperando la activación de uno de los eventos, que corresponde con la recepción de un comando a través de la uart
		e = k_event_wait(&event, scenario_1|scenario_2|scenario_3|test_started|test_finished|per_1|per_2, true, K_FOREVER);

		LOG_INF("Hebra desbloqueada");

		if((e & scenario_1) != 0){ //Cambio de escenario
			LOG_INF("Escenario de prueba 1");
			scenario = scenario_1;			
		}

		if((e & scenario_2) != 0){ //Cambio de escenario
			LOG_INF("Escenario de prueba 2");
			scenario = scenario_2;		
		}

		if((e & scenario_3) != 0){ //Cambio de escenario
			LOG_INF("Escenario de prueba 3");
			scenario = scenario_3;	
		}

		if((e & test_started) != 0){ //Se inicia la prueba
			LOG_INF("Inicio de la prueba");

			memset(its_buf+2, '\0', TAMA_BUF); 

			LOG_INF("---------------");
			LOG_INF("Trama número %d", contador);
			memcpy(its_buf+2, &contador, sizeof(uint16_t));
			uint8_t aux = its_buf[2];
			its_buf[2] = its_buf[3];
			its_buf[3] = aux;
			contador++;

			err = clock_gettime(CLOCK_REALTIME, &time_now);
			if(err){
				LOG_ERR("clock_gettime error (%d)", err);
				while(1);
			}

			unix_time_ns = time_now.tv_sec * 1E9 + time_now.tv_nsec;	
			LOG_INF("unix_time_ns: %lld", unix_time_ns);		
			memcpy(its_buf+4, &unix_time_ns, sizeof(int64_t));

			LOG_INF("---------------");

			//corregimos el efecto del memcpy, que altera el orden de los bytes
			/*for(int i = 0; i < 4; i++){
				aux = its_buf[i + 4];
				its_buf[i + 4] = its_buf[11 - i];
				its_buf[11 - i] = aux;
			}*/

			its_buf[12] = DEVICE_ID;
			//Según el escenario en el que se realice la prueba el intervalo inicial y la trama que se emite es distinta
			switch(scenario){
				case scenario_1: 
					memcpy(its_buf+13, non_secure_frame, sizeof(non_secure_frame));
					int_min = 0x0500; //800 ms intervalo minimo
					int_max = 0x0640; //1 s intervalo máximo
				break;

				case scenario_2:
					memcpy(its_buf+13, non_secure_frame, sizeof(non_secure_frame));
					int_min = 0x0500; //800 ms intervalo minimo
					int_max = 0x0640; //1 s intervalo máximo
				break;

				case scenario_3:
					memcpy(its_buf+13, hash_frame, sizeof(hash_frame));
					int_min = BT_GAP_ADV_FAST_INT_MIN_2; //100 ms de intervalo mínimo
					int_max = BT_GAP_ADV_FAST_INT_MAX_2; //150 ms de intervalo máximo
				break;
			}

			//Actualizo adv parameters de acuerdo al escenario
			adv_param = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, int_min, int_max, NULL); 
			err = bt_le_ext_adv_update_param(adv, adv_param);
			if(err < 0){
				LOG_ERR("(bt_le_ext_adv_update_param) error %d \n", err);
				while(1);
			}

			//Actualización de los datos de advertisement			
			err = bt_le_ext_adv_set_data(adv, ad_its, ARRAY_SIZE(ad_its), NULL, 0);
			if (err < 0) {
				LOG_ERR("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err);
			}

			if(scenario == scenario_3){
				//Se inicia el timer periódico para enviar de forma periódica el certificado
				k_timer_start(&timer, K_MSEC(PER), K_MSEC(PER));
			}

			irq_enable(SWI1_IRQn);

			//Start extended advertising	
			//Advertising interval is the time between each Advertise Event.
			//BT_LE_EXT_ADV_START_DEFAULT equivale a no timeout, no limit of advertisement events
			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err) {
				LOG_ERR("(bt_le_ext_adv_starting) Advertising start failed (err %d)\n", err);
				while(1);
			}
		}

		if((e & test_finished) != 0){
		
			LOG_INF("Stop advertisement");
			err = bt_le_ext_adv_stop(adv);
			if(err < 0){
				LOG_ERR("(bt_le_ext_adv_stop) error %d \n", err);
				while(1);
			}

			irq_disable(SWI1_IRQn);

			k_timer_stop(&timer);

			contador = 0;

			LOG_INF("Final");
		}

		if((e & per_1) != 0){
			LOG_INF("Periodo de advertising 1");
			
			int err = bt_le_ext_adv_stop(adv);
			if(err < 0){
				LOG_ERR("(bt_le_ext_adv_stop) error %d \n", err);
				while(1);
			}

			switch(scenario){
				case scenario_1:
					int_min = 0x0500; //0,8 segundos
					int_max = 0x0640; //1 segundo
				break;

				case scenario_2:
					int_min = 0x0500; //0,8 segundos
					int_max = 0x0640; //1 segundo
				break;

				case scenario_3:
					int_min = BT_GAP_ADV_FAST_INT_MIN_1; //30 ms
					int_max = BT_GAP_ADV_FAST_INT_MAX_1; //60 ms
				break;

			}

			contador = 0;

			LOG_INF("Intervalo de advertisement %d %d", int_min*0.625, int_max*0.625);
			
			adv_param = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, int_min, int_max,NULL); 
			err = bt_le_ext_adv_update_param(adv, adv_param);
			if(err < 0){
				LOG_ERR("(bt_le_ext_adv_update_param) error %d \n", err);
				while(1);
			}

			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err) {
				LOG_ERR("(bt_le_ext_adv_starting) Advertising start failed (err %d)\n", err);
				while(1);
			}
		}

		if((e & per_2) != 0){
			LOG_INF("Periodo de advertising 2");
			
			int err = bt_le_ext_adv_stop(adv);
			if(err < 0){
				LOG_ERR("(bt_le_ext_adv_stop) error %d \n", err);
				while(1);
			}
			
			switch(scenario){
				case scenario_1:
					int_min = BT_GAP_ADV_FAST_INT_MIN_2; //100 ms
					int_max = BT_GAP_ADV_FAST_INT_MAX_2; //150 ms
				break;

				case scenario_2:
					int_min = BT_GAP_ADV_FAST_INT_MIN_2; //100 ms
					int_max = BT_GAP_ADV_FAST_INT_MAX_2; //150 ms
				break;

				case scenario_3:
					int_min = 0x50; //BT_GAP_ADV_FAST_INT_MIN_1; //30 ms (pruebo con 50ms)
					int_max = 0xa0; //BT_GAP_ADV_FAST_INT_MAX_1; //60 ms (pruebo con 100 ms)
				break;

			}

			contador = 0;

			LOG_INF("Intervalo de advertisement %d %d", int_min*0.625, int_max*0.625);
			
			adv_param = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, int_min, int_max,NULL); 
			err = bt_le_ext_adv_update_param(adv, adv_param);
			if(err < 0){
				LOG_ERR("(bt_le_ext_adv_update_param) error %d \n", err);
				while(1);
			}

			err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
			if (err) {
				LOG_ERR("(bt_le_ext_adv_starting) Advertising start failed (err %d)\n", err);
				while(1);
			}
		}
	}
}
