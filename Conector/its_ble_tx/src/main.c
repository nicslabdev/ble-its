#include <zephyr.h>
#include <drivers/uart.h>
#include <device.h>
#include <soc.h>

#include <kernel.h>
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
#include <usb/usb_device.h>


#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define LOG_MODULE_NAME ble_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);


//UUID del servicio
#define CUSTOM_SERVICE_UUID BT_UUID_128_ENCODE(0xe7510001, 0xed0f, 0x492b, 0x80e8, 0xa2226aef72b3)

#define TAMA_BUF 238 //tamaño máximo de datos sin que haya fragmentación
uint8_t ble_buf[TAMA_BUF+2]; //buffer que se envía por BLE

typedef struct{
	uint8_t b[TAMA_BUF];
	uint8_t tama;
}Tbuf;

Tbuf uart_buf;
static const struct device *uart;

struct bt_data ad[3] = { 
	//BT_DATA_BYTES(type of advertising data field, single-byte parameters)
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR|BT_LE_AD_GENERAL), //BR/EDR not supported, general discoverable (no timeout)
	//BT_DATA_BYTES(BT_DATA_UUID128_ALL, CUSTOM_SERVICE_UUID),
	BT_DATA_BYTES(BT_DATA_NAME_SHORTENED, DEVICE_NAME),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, ble_buf, ARRAY_SIZE(ble_buf)) 
};
struct bt_le_ext_adv *adv;

void button_callback(uint32_t button_state, uint32_t has_changed);
static void uart_irq_handler(const struct device *dev, void *context);

typedef enum states {one_pressed, two_pressed, three_pressed};
uint8_t state;

void main(void){
	int err;

	//inicializacion de los leds
	err = dk_leds_init();
	if(err < 0){
		printk("(dk_leds_init) error %d", err);
	}
	
	err = dk_buttons_init(button_callback);
	if(err < 0){
		printk("(dk_buttons_init) error %d", err);
	}

	//estado inicial
	state = one_pressed;
	dk_set_leds(DK_LED1_MSK);


	//inicialización del logger
	log_init();
	LOG_INF("Starting BLE-ITS (TX)\n\r");

	//Inicialización módulo UART
	uart = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(uart)) {
		LOG_ERR("CDC ACM device not ready");
		return;
	}

	err = usb_enable(NULL);
	if (err != 0) {
		LOG_ERR("Failed to enable USB");
		return;
	}

	//Inicialización sistema Bluetooth
	err = bt_enable(NULL); 
	if (err) {
		printk("bt_enable: error %d\n", err);
		while(1);
	}

	printk("Bluetooth initialized\n");

	//ADVERTISEMENT EXTENDIDO
	struct bt_le_adv_param* adv_param = 
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, //advertising non-connectable y non-scannable (esto ultimo quiere decir que no soporta un mensaje Scan Request)
		BT_GAP_ADV_SLOW_INT_MIN, //intervalo de advertising mínimo 
		BT_GAP_ADV_SLOW_INT_MAX, //intervalo de advertising máximo
		NULL); //Dirección del peer: NULL porque el advertising es undirected

	/*
	//Info sobre el advertising interval: valores que se incrementan en 0.625 ms
	#define BT_GAP_ADV_FAST_INT_MIN_1               0x0030  // 30 ms    
	#define BT_GAP_ADV_FAST_INT_MAX_1               0x0060  // 60 ms    
	#define BT_GAP_ADV_FAST_INT_MIN_2               0x00a0  // 100 ms   
	#define BT_GAP_ADV_FAST_INT_MAX_2               0x00f0  // 150 ms   
	#define BT_GAP_ADV_SLOW_INT_MIN                 0x0640  // 1 s      
	#define BT_GAP_ADV_SLOW_INT_MAX                 0x0780  // 1.2 s    
	*/

	err = settings_load();
	if(err < 0){
		printk("(settings_load) error %d \n", err);
		while(1);
	}

	/* Create an advertising set */
	err = bt_le_ext_adv_create(adv_param, NULL, &adv);
	if (err < 0) {
		printk("(bt_le_ext_adv_create) Advertising set creation failed (err %d)\n", err);
		while(1);
	}

	//Inicialización de los buffers
	//los dos primeros bytes indican el company ID
	//0xffff company ID is reserved for testing porpuses only and cannot be used for shipping commercial products
	//0x0059 -> Nordics Company ID
	ble_buf[0] = 0x59;
	ble_buf[1] = 0x00;
	memset(ble_buf+2, '\0', TAMA_BUF); 

	/*Set advertising data*/
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err < 0) {
		printk("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err);
		while(1);
	}

	/*Start extended advertising*/	
	//BT_LE_EXT_ADV_START_DEFAULT equivale a no timeout, no limit of advertisement events
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("(bt_le_ext_adv_starting) Advertising start failed (err %d)\n", err);
		while(1);
	}

	//Establezco ISR
	uart_irq_callback_set(uart, uart_irq_handler);
	//Habilito las interrupciones
	uart_irq_rx_enable(uart);

}


bool inicio = true;
size_t tama = 0;

static void uart_irq_handler(const struct device *dev, void *context)
{
	uint8_t c, err;
	volatile bool fin = false;
	printk("Entrada al ISR\n");

	if(uart_buf.tama == tama && tama != 0){
		printk("Había llegado la trama completa!! (uart_buf.tama %ld), (tama %ld) \n", uart_buf.tama, tama);
		memcpy(ble_buf+2, uart_buf.b, TAMA_BUF); //los datos recogidos se copian al buffer de BLE 

		err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
		if (err < 0) {
			printk("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err);
			while(1);
		}

		//El buffer de la UART se reinicia
		uart_buf.tama = 0;
		memset(uart_buf.b+uart_buf.tama, '\0', TAMA_BUF-uart_buf.tama); 
		fin = false;
		inicio = true;
	}

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			err = uart_fifo_read(dev, &c, sizeof(uint8_t)); 
			if(err <= 0){
				printk("(uart_fifo_read) error %d", err);
			}else{
				
				if(inicio == true){
					//Si es el inicio de la trama voy a obtener el tamaño
					printk("Tamaño de la trama: %d", c);
					tama = c + 1;
					uart_buf.b[uart_buf.tama] = c;
					uart_buf.tama++;
					inicio = false;
				}else{
					printk("Received: %02x\n", c);	
					uart_buf.b[uart_buf.tama] = c;
					uart_buf.tama++;
					printk("Recibido, tamaño actual del buffer %d\n", uart_buf.tama);
				}

				if(uart_buf.tama == TAMA_BUF){ 
					//Para no desbordar el buffer
					printk("Buffer lleno\n");
					fin = true;
				}

				if(uart_buf.tama == tama){
					printk("Ha llegado la trama completa (uart_buf.tama %ld), (tama %ld) \n", uart_buf.tama, tama);
					fin = true;
				}

				if(fin){
					printk("Se termina de leer: fin o buffer lleno (tamaño %d) \n", uart_buf.tama);
					memcpy(ble_buf+2, uart_buf.b, TAMA_BUF); //los datos recogidos se copian al buffer de BLE 
						
					err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
					if (err < 0) {
						printk("(bt_le_ext_adv_set_data) Set advertising set failed (err %d)\n", err);
						while(1);
					}

					//El buffer de la UART se reinicia
					uart_buf.tama = 0;
					memset(uart_buf.b+uart_buf.tama, '\0', TAMA_BUF-uart_buf.tama); 
					fin = false;
					inicio = true;
				}
			}
		}
	}
}


void button_callback(uint32_t button_state, uint32_t has_changed){
	//printk("Button callback\n");
	struct bt_le_adv_param* adv_param;
	int int_min,int_max;
	if(((has_changed & DK_BTN1_MSK) != 0) && (button_state)){
		//Se ha presionado el botón 1
		//printk("button_state %08x has_changed %08x\n", button_state, has_changed);
		switch(state){
			case one_pressed:
				printk("State transition: intervalo de advertising 100-150 ms %02x %02x \n", BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2);
				state = two_pressed;
				dk_set_leds(DK_LED2_MSK);
				int_min = BT_GAP_ADV_FAST_INT_MIN_2;
				int_max = BT_GAP_ADV_FAST_INT_MAX_2;
			break;

			case two_pressed:
				printk("State transition: intervalo de advertising 30-60 ms %02x %02x\n", BT_GAP_ADV_FAST_INT_MIN_1, BT_GAP_ADV_FAST_INT_MAX_1);
				state = three_pressed;
				dk_set_leds(DK_LED3_MSK);
				int_min = BT_GAP_ADV_FAST_INT_MIN_1;
				int_max = BT_GAP_ADV_FAST_INT_MAX_1; 
			break;

			case three_pressed:
				printk("State transition: intervalo de advertising 1-1.02 s %02x %02x\n", BT_GAP_ADV_SLOW_INT_MIN, BT_GAP_ADV_SLOW_INT_MAX);
				state = one_pressed;
				dk_set_leds(DK_LED1_MSK);
				int_min = BT_GAP_ADV_SLOW_INT_MIN;
				int_max = BT_GAP_ADV_SLOW_INT_MAX;
			break;

			default:
				printk("Error\n");
				while(1);
			break;
		}

		int err = bt_le_ext_adv_stop(adv);
		if(err < 0){
			LOG_ERR("(bt_le_ext_adv_stop) error %d \n", err);
			while(1);
		}

		printf("%02x %02x \n", int_min, int_max);

		adv_param = BT_LE_ADV_PARAM(BT_LE_ADV_OPT_EXT_ADV, int_min, int_max,NULL); 
		err = bt_le_ext_adv_update_param(adv, adv_param);
		if(err < 0){
			LOG_ERR("(bt_le_ext_adv_update_param) error %d \n", err);
			while(1);
		}

		err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
		if (err) {
			printk("(bt_le_ext_adv_starting) Advertising start failed (err %d)\n", err);
			while(1);
		}

	}
}
