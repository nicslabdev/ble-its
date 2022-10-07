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
#include <bluetooth/conn.h>
#include <bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/services/cts_client.h>
#include <bluetooth/uuid.h>
#include <bluetooth/scan.h>
#include <settings/settings.h>
#include <stdio.h>
#include <logging/log.h>
#include <logging/log_core.h>
#include <logging/log_ctrl.h>
#include <usb/usb_device.h>
#include "time.h"
#include <dk_buttons_and_leds.h>
#include <device.h>
#include <drivers/gpio.h>
#include <date_time.h>
#include <sys/timeutil.h>

// LOG CONFIG PARAMETERS
// ------------------------------------------------
#define LOG_MODULE_NAME ble_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);
// ------------------------------------------------

// GLOBAL VARIABLES
// ------------------------------------------------
const struct device *uart;
// ------------------------------------------------


// IMPLEMENTED FUNCTIONS
// ------------------------------------------------
void print_filter_match(struct bt_scan_filter_match *filter_match);
void print_scan_info(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type, struct net_buf_simple *buf, bool connectable);
static void scan_filter_match(struct bt_scan_device_info *device_info, struct bt_scan_filter_match *filter_match, bool connectable);
void scan_callback(const bt_addr_le_t *addr, int8_t rssi,uint8_t adv_type, struct net_buf_simple *buf);
static bool parse_frames(struct bt_data *data, void *user_data);
// ------------------------------------------------

//BLE CONFIG PARAMETERS
// ------------------------------------------------
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

//ESCANEO
//Se establece el callback que se ejecuta cuando un advertisement hace match con el filtro
char short_name[4] = {'I', 'T', 'S', '\0'};
BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL, NULL, NULL);
// ------------------------------------------------



void main(void)
{
	int err;

	//inicialización del logger
	log_init();
	LOG_INF("Starting BLE-ITS (RX)\n\r");


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

	//Si el emisor como mucho va a a anunciar cada 30ms pongo como intervalo de escaneo 30ms
		struct bt_scan_init_param scan_init = {
		.scan_param = &scan_param,
		.connect_if_match = 0,
		.conn_param = NULL
	};


	LOG_INF("Inicio escaneo");
	bt_scan_init(&scan_init);
	bt_scan_cb_register(&scan_cb);

	//Se establece un filtro en base al SHORT NAME
	struct bt_scan_short_name bt_scan_sn;
	bt_scan_sn.name = short_name;
	bt_scan_sn.min_len = 4;

	LOG_INF("Name: %s, sizeof)%d", bt_scan_sn.name, sizeof(bt_scan_sn.name));

	err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_SHORT_NAME, &bt_scan_sn);
	if (err < 0) {
		LOG_ERR("Scanning filters cannot be set (err %d)", err);
		while(1);
	}

	err = bt_scan_filter_enable(BT_SCAN_SHORT_NAME_FILTER, false);
	if (err < 0) {
		LOG_ERR("Filters cannot be turned on (err %d)", err);
		while(1);
	}


	err = bt_le_scan_start(&scan_param, NULL);
	if(err < 0){
		LOG_ERR("Scanning failed to start (%d)", err);
	}

	LOG_INF("Scan module initialized");	
		

}


static void scan_filter_match(struct bt_scan_device_info *device_info,
			      struct bt_scan_filter_match *filter_match,
			      bool connectable)
{

	LOG_INF("----------------");
	LOG_INF("Filter matched");
	print_scan_info(device_info->recv_info->addr, device_info->recv_info->rssi, device_info->recv_info->adv_type, device_info->adv_data, connectable);
	print_filter_match(filter_match);
	LOG_INF("Advertising data");
	//envío a la pila la información de advertising
	uart_poll_out(uart, device_info->adv_data->len);
	for(int i = 0; i < device_info->adv_data->len; i++){
		uart_poll_out(uart, device_info->adv_data->data[i]);
	}
	
	bt_data_parse(device_info->adv_data, parse_frames, NULL);
	LOG_INF("----------------\n");
}

static bool parse_frames(struct bt_data *data, void *user_data){
	int err;

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
		break;

		case BT_DATA_NAME_SHORTENED:
			LOG_INF("	-> Shortened name");
		break;
	}

	return true;
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

	//Si el filtro está basado en UUID
	/*char uuid_str[37];
	memset(uuid_str, '\0', sizeof(uuid_str));
	for(int i=0; i<filter_match->uuid.len; i++){
			bt_uuid_to_str(filter_match->uuid.uuid[i], uuid_str, sizeof(uuid_str));		
			memset(uuid_str, '\0', sizeof(uuid_str));
	}*/
}
