#include <zephyr.h>
#include <sys/byteorder.h>
#include <drivers/uart.h>
#include <soc.h>
#include <kernel.h>
#include <sys/printk.h>
#include <stddef.h>
#include <sys/util.h>
#include <errno.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/scan.h>
#include <settings/settings.h>
#include <stdio.h>
#include <logging/log.h>
#include <logging/log_core.h>
#include <logging/log_ctrl.h>
#include <usb/usb_device.h>
#include <time.h>
#include <dk_buttons_and_leds.h>
#include <device.h>
#include <drivers/gpio.h>
#include <date_time.h>
#include <sys/timeutil.h>

// TIME
#define DELTA 0 //ms

// LOG CONFIG PARAMETERS
// ------------------------------------------------
#define LOG_MODULE_NAME its_ble_rx_poc
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);
// ------------------------------------------------

// DEFINED ENUMS
// ------------------------------------------------
enum commands_9160_52840 {unblock_threads = 0x01, set_first = 0x02};
enum commands_52840_9160{first_received = 0x01};
// ------------------------------------------------


// IMPLEMENTED FUNCTIONS
// ------------------------------------------------
void print_filter_match(struct bt_scan_filter_match *filter_match);
void print_scan_info(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf, bool connectable);
static void scan_filter_match(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match, bool connectable);
static bool parse_frames(struct bt_data *data, void *user_data);
void change_endianess_timestamp(uint8_t *buf);
void change_endianess_id(uint8_t* buf);
void obtain_frame_timestamp_its(struct bt_data *data, int64_t *frame_time_ns);
uint16_t obtain_frame_id(struct bt_data *data);
uint8_t obtain_frame_size(struct bt_data *data);
uint8_t obtain_device_id(struct bt_data *data);

void send_command(unsigned char m);
static void uart1_irq_handler(const struct device *dev, void *context);

void hebra(void);
void hebra2(void);

void scan_callback(const bt_addr_le_t *addr, int8_t rssi,uint8_t adv_type, struct net_buf_simple *buf);

// ------------------------------------------------


//BLE CONFIG PARAMETERS
// ------------------------------------------------
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

//ESCANEO
//Se establece el callback que se ejecuta cuando un advertisement hace match con el filtro
char short_name_its[4] = {'I', 'T', 'S', '\0'};
BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, NULL);
// ------------------------------------------------


// GLOBAL VARIABLES
// ------------------------------------------------
static const struct device *uart;
static const struct device *uart1;

char out[500];

typedef struct{
    uint16_t frame_id;
	uint16_t device_id;
    int64_t unix_tx;
	int64_t unix_rx;
    uint8_t tama;
}TMuestra;

typedef TMuestra TMuestras[500];

typedef struct{
	TMuestras a;
	int tama;
}TArray;

TArray v1;
TArray v2;

bool permitido = false;
bool first = true;
int vector = 1;
int64_t unix_time_rx;
// ------------------------------------------------


// THREAD CONFIG PARAMETERS
// ------------------------------------------------
/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 5

K_THREAD_DEFINE(h, STACKSIZE, hebra, NULL, NULL, NULL, PRIORITY, 0, 0);
K_THREAD_DEFINE(h2, STACKSIZE, hebra2, NULL, NULL, NULL, PRIORITY, 0, 0);
// ------------------------------------------------

// SEMAPHORE
// ------------------------------------------------
K_SEM_DEFINE(uart_sem, 0, 1);
K_SEM_DEFINE(uart2_sem, 0, 1);
// ------------------------------------------------


void main(void)
{

	int err;
	unsigned char c;
	uint8_t buf[8]; //Se almacena el tiempo que se recibe a través de la uart
	int64_t unix_time_ns;
	int aux = 0;
	struct timespec received_time;


	v1.tama = 0;
	v2.tama = 0;
	
	//inicialización del logger
	log_init();
	LOG_INF("Starting BLE-ITS (RX)\n\r");


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


	//Inicialización módulo UART: UART over USB
	uart = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
	if (!device_is_ready(uart)) {
		LOG_ERR("CDC ACM device not ready");
		while(1);
	}

	err = usb_enable(NULL);
	if (err != 0) {
		LOG_ERR("Failed to enable USB");
		while(1);
	}

	// Establezco ISR de la UART
	uart_irq_callback_set(uart1, uart1_irq_handler);
	// Se habilitan las interrupciones
	uart_irq_rx_enable(uart1);

	//Inicialización Bluetooth
	err = bt_enable(NULL); 
	if (err) {
		LOG_ERR("bt_enable: error %d\n", err);
		while(1);
	}

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	//Parámetros de escaneo
	struct bt_le_scan_param scan_param = { //Parametros de escaneo
		.type       = BT_LE_SCAN_TYPE_PASSIVE, //tipo de escaneo
		.options    = BT_LE_SCAN_OPT_FILTER_ACCEPT_LIST, //opciones escaneo
		.interval   = 0x0030, //intervalo de escaneo (48 * 0.625 ms) = 30 ms
		.window     = 0x0030, //ventana de escaneo (48 * 0.625 ms) = 30 ms
	};

	struct bt_scan_init_param scan_init = {
		.scan_param = &scan_param,
		.connect_if_match = 0,
		.conn_param = NULL
	};

	LOG_INF("Inicio escaneo");
	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);
	
	struct bt_scan_short_name bt_scan_sn;
	bt_scan_sn.name = short_name_its;
	bt_scan_sn.min_len = 4;

	LOG_INF("Name: %s, sizeof)%d", bt_scan_sn.name, sizeof(bt_scan_sn.name));

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_SHORT_NAME, &bt_scan_sn);
	if (err < 0)
	{
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		while (1);
	}

	err = bt_scan_filter_enable(BT_SCAN_SHORT_NAME_FILTER, false);
	if (err < 0)
	{
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		while (1);
	}

	permitido = true;

	k_sleep(K_MSEC(2000)); //se duerme el main antes de comenzar el escaneo para que las hebras se puedan iniciar

	LOG_INF("Scan module initialized");	
	err = bt_le_scan_start(&scan_param, scan_callback);
	if (err < 0)
	{
		LOG_ERR("Scanning failed to start (%d)", err);
	}

}


void print_scan_info(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf, bool connectable){
	char print_addr[BT_ADDR_LE_STR_LEN];

	switch(addr->type){
		case BT_ADDR_LE_PUBLIC:
			LOG_INF("Address type:public");
		break;
		case BT_ADDR_LE_RANDOM:
			LOG_INF("Address type:random");
		break;
		case BT_ADDR_LE_PUBLIC_ID:
			LOG_INF("Address type:public_id");
		break;
		case BT_ADDR_LE_RANDOM_ID:
			LOG_INF("Address type:random_id");
		break;
		case BT_ADDR_LE_UNRESOLVED:
			LOG_INF("Address type:unresolved");
		break;
		case BT_ADDR_LE_ANONYMOUS:
			LOG_INF("Address type:anonymous");
		break;
	}

	bt_addr_le_to_str(addr, print_addr, sizeof(print_addr));

	LOG_INF("Address: %s",log_strdup(print_addr));

	LOG_INF("Connectable: %d", connectable);

	LOG_INF("RSSI: %d ",rssi);
	switch(adv_type){
		case BT_GAP_ADV_TYPE_ADV_IND:
			LOG_INF("Type: Scannable and connectable advertising");
		break;
		case BT_GAP_ADV_TYPE_ADV_DIRECT_IND :
			LOG_INF("Type: Directed connectable advertising");
		break;
		case BT_GAP_ADV_TYPE_ADV_SCAN_IND:
			LOG_INF("Type: Non-connectable and scannable advertising");
		break;
		case BT_GAP_ADV_TYPE_ADV_NONCONN_IND:
			LOG_INF("Type:Non-connectable and non-scannable advertising");
		break;
		case BT_GAP_ADV_TYPE_SCAN_RSP:
			LOG_INF("Type: Additional advertising data requested by an active scanner");
		break;
		case BT_GAP_ADV_TYPE_EXT_ADV:
			LOG_INF("Type: Extended advertising");
		break;
	}

	LOG_INF("The buffer can store: %d bytes", net_buf_simple_max_len(buf));
	LOG_INF("Advertising data length: %d", buf->len);
}

void print_filter_match(struct bt_scan_filter_match *filter_match){
	LOG_INF("Short Name filter matched: %s",filter_match->short_name.name);
}



static bool parse_frames(struct bt_data *data, void *user_data){
	struct timespec time_struct;
	int64_t frame_time_ns;
    TMuestra m;
	int err;
	char name[10];

	switch(data->type){
		case BT_DATA_FLAGS:
			LOG_INF("	-> Flags: ");

			if((*(data->data) & BT_LE_AD_LIMITED) != 0){
				LOG_INF("		Limited Discoverable");
			}
			if((*(data->data) & BT_LE_AD_GENERAL) != 0){
				LOG_INF("		General Discoverable");
			}	

			if((*(data->data) & BT_LE_AD_NO_BREDR) != 0){
				LOG_INF("		BR/EDR not supported");
			}	

		break;

		case BT_DATA_MANUFACTURER_DATA:
			LOG_INF("	-> Manufacturer data (%d bytes)", data->data_len);

			//Obtengo el tamaño de la trama ITS
			m.tama = obtain_frame_size(data);
			LOG_INF("		Tamaño de trama: %d", m.tama);
			
			//Obtengo el número de trama ITS
			m.frame_id = obtain_frame_id(data);
			LOG_INF("		Identificador de trama: %d", m.frame_id);

			//Obtengo el identificador de dispositivo transmisor
			m.device_id = obtain_device_id(data);
			LOG_INF("		Identificador de dispositivo: %d", m.device_id);
			if((m.device_id == 1) && (first)){
				send_command(first_received);
				first = false;
			}
			
			//Obtengo el instante de transmisión del mensaje
			obtain_frame_timestamp_its(data, &frame_time_ns);

			m.unix_tx = frame_time_ns;
			m.unix_rx = unix_time_rx;

			switch(vector){
				case 1:
					LOG_INF("vector 1");
					v1.a[v1.tama] = m;
					v1.tama++;

					if(v1.tama == 399){ //está lleno
						k_sem_give(&uart_sem);
						vector = 2;
					}

				break;

				case 2:
					LOG_INF("vector 2");
					v2.a[v2.tama] = m;
					v2.tama++;

					if(v2.tama == 399){ //está lleno
						k_sem_give(&uart2_sem);
						vector = 1;
					}
				break;
			}
		break;

		case BT_DATA_NAME_SHORTENED:
			LOG_INF("	-> Shortened name");
		break;
	}

	return true;
}

uint16_t obtain_frame_id(struct bt_data *data){
	uint16_t frame_num;
	uint8_t num[2] = {0x00, 0x00};

	memcpy(num, data->data+2, sizeof(num));
	change_endianess_id(num);
	memcpy(&frame_num, num, sizeof(uint16_t));

	return frame_num;
}

uint8_t obtain_device_id(struct bt_data *data){
    //Se obtiene fácilmente conociendo la posición de la trama en la que se coloca
    uint8_t s = *(data->data+12);

    return s;
}

void obtain_frame_timestamp_its(struct bt_data *data, int64_t *frame_time_ns){
	int64_t frame_time_s;
	uint8_t time[8];
	struct tm* time_struct;

	//memset(time, 0x00, sizeof(time));
	//memcpy(time, data->data+4, sizeof(time));
	//change_endianess_timestamp(time);
	//memcpy(frame_time_ms, time, sizeof(int64_t));
	
	memcpy(frame_time_ns, data->data+4, sizeof(int64_t));
}


uint8_t obtain_frame_size(struct bt_data *data){
    //Se obtiene fácilmente conociendo la posición de la trama en la que se coloca
    uint8_t s = *(data->data+13);

    return s;
}

//memcpy da la vuelta a los bytes, con estas dos funciones se corrige

void change_endianess_id(uint8_t* buf){
	uint8_t aux;
	aux = *buf;
	*buf = *(buf+1);
	*(buf+1) = aux;
}

void change_endianess_timestamp(uint8_t* buf){
	uint8_t aux;

	for(int i = 0; i < 4; i++){
		aux = *(buf+i);
		*(buf+i) = *(buf+7-i);
		*(buf+7-i) = aux;
	}
}

static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{
	int err;
	LOG_INF("----------------");
	LOG_INF("Filter matched");
	print_scan_info(device_info->recv_info->addr, device_info->recv_info->rssi, device_info->recv_info->adv_type, device_info->adv_data, connectable);
	print_filter_match(filter_match);
	LOG_INF("Advetising data");
	bt_data_parse(device_info->adv_data, parse_frames, NULL);
	LOG_INF("----------------\n");
}

void hebra(void){
	int err;
	uint32_t dtr = 0;

	LOG_INF("Se inicia la hebra 1\n");

	//Para evitar enviar la primera linea del csv sin que se haya abierto la consola de putty
	while (!dtr) {
		uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	sprintf(out, "frame_id,size,unix_tx,unix_rx,device_id\r\n");
    for(int j = 0; j < strlen(out); j++){
        uart_poll_out(uart, out[j]);
    }   

	while(1){
		err = k_sem_take(&uart_sem, K_FOREVER);

        if(err == 0){
            LOG_INF("Hebra 1 desbloqueada\n");
    
			for(int i = 0; i < v1.tama; i++){
                sprintf(out, "%d, %d, %"PRId64", %"PRId64",%d\r\n", v1.a[i].frame_id, v1.a[i].tama, v1.a[i].unix_tx, v1.a[i].unix_rx, v1.a[i].device_id);
				for(int j = 0; j < strlen(out); j++){
                    uart_poll_out(uart, out[j]);
                }
            }

			memset(v1.a, 0, 400);
			v1.tama = 0;
        }
    }
}



void hebra2(void){
	int err;

	LOG_INF("Se inicia la hebra 2\n");

	while(1){
		err = k_sem_take(&uart2_sem, K_FOREVER);

        if(err == 0){
            LOG_INF("Hebra 2 desbloqueada\n");
        
			for(int i = 0; i < v2.tama; i++){
                sprintf(out, "%d, %d, %"PRId64", %"PRId64",%d\r\n", v2.a[i].frame_id, v2.a[i].tama, v2.a[i].unix_tx, v2.a[i].unix_rx,v2.a[i].device_id);
				for(int j = 0; j < strlen(out); j++){
                    uart_poll_out(uart, out[j]);
                }
            }

			memset(v2.a, 0, 400);
			v2.tama = 0;
        }
    }
}

void scan_callback(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf){
	//Intento almacenar el instante de tiempo más cercano al real actualizando la variable cada vez que hay un evento relacionado con el escaneo
	struct timespec t;
	int err = clock_gettime(CLOCK_REALTIME, &t);
	if(err){
		LOG_ERR("clock_gettime error (%d)", err);
		while(1);
	}

	unix_time_rx = t.tv_sec * 1E9 + t.tv_nsec;

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

				if((c & unblock_threads) != 0){
					k_sem_give(&uart_sem);
					k_sem_give(&uart2_sem);
				}

				if((c & set_first) != 0){
					first = true;
				}
			}
		}
	}
}

void send_command(unsigned char m){
    uart_poll_out(uart1, m);
}
