#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <dk_buttons_and_leds.h>
#include <logging/log.h>
#include <logging/log_core.h>
#include <logging/log_ctrl.h>
#include <drivers/uart.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <modem/lte_lc.h>
#include "time.h"

// LOG CONFIG PARAMETERS
// ------------------------------------------------
#define LOG_MODULE_NAME advertising_config
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);
// ------------------------------------------------


// DEVICE TREE NODES 
// ------------------------------------------------
#define RED_NODE DT_ALIAS(led0)
#define GREEN_NODE DT_ALIAS(led1)
#define BLUE_NODE DT_ALIAS(led2)

#if DT_NODE_HAS_STATUS(RED_NODE, okay)
#define LED0	DT_GPIO_LABEL(RED_NODE, gpios)
#define PIN0	DT_GPIO_PIN(RED_NODE, gpios)
#define FLAGS0	DT_GPIO_FLAGS(RED_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN0	0
#define FLAGS0	0
#endif

#if DT_NODE_HAS_STATUS(GREEN_NODE, okay)
#define LED1	DT_GPIO_LABEL(GREEN_NODE, gpios)
#define PIN1	DT_GPIO_PIN(GREEN_NODE, gpios)
#define FLAGS1	DT_GPIO_FLAGS(GREEN_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led1 devicetree alias is not defined"
#define LED1	""
#define PIN1	0
#define FLAGS1	0
#endif

#if DT_NODE_HAS_STATUS(BLUE_NODE, okay)
#define LED2	DT_GPIO_LABEL(BLUE_NODE, gpios)
#define PIN2	DT_GPIO_PIN(BLUE_NODE, gpios)
#define FLAGS2	DT_GPIO_FLAGS(BLUE_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led2 devicetree alias is not defined"
#define LED2	""
#define PIN2	0
#define FLAGS2	0
#endif


static const struct device *uart1;
static const struct device *leds[3];
// ------------------------------------------------


// EVENTS
// ------------------------------------------------
K_EVENT_DEFINE(event);
// ------------------------------------------------

// SEMAPHORES
// ------------------------------------------------
static K_SEM_DEFINE(time_sem, 0, 1);
static K_SEM_DEFINE(first_sem, 0, 1);
// ------------------------------------------------

typedef struct TFix{
	double latitude;
	double longitude;
	uint8_t minute;
	uint8_t seconds;
	uint16_t ms;
}TFix;


// IMPLEMENTED FUNCTIONS 
// ------------------------------------------------
void button_callback(uint32_t button_state, uint32_t has_changed);
void hebra(void);
static void timer_callback(struct k_timer *timer);
static void blink_callback(struct k_timer *timer);

int leds_init();
void blink_indication(bool red, bool green, bool blue);

static void uart1_irq_handler(const struct device *dev, void *context);
void send_command(unsigned char m);
void serializar(int64_t time, uint8_t* ser);

static void gnss_event_handler(int e);
static int gnss_init_and_start(void);
static int modem_init(void);
static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data);
// ------------------------------------------------

// TIMER
// ------------------------------------------------
#define PER 1000 //ms
K_TIMER_DEFINE(timer_button_pressed, timer_callback, NULL);

#define PER_BLINK 500 //ms
K_TIMER_DEFINE(timer_blink, blink_callback, NULL);
//-------------------------------------------------


// DEFINED ENUMS
// ------------------------------------------------
enum states {searching_fix, scan};
enum gnss_events {fix_obtained = 0x01};
enum commands_9160_52840 {unblock_threads = 0x01, set_first = 0x02};
enum commands_52840_9160{first_received = 0x01};
enum { RED, GREEN, BLUE };
// ------------------------------------------------


// GLOBAL VARIABLES
// ------------------------------------------------
uint8_t state = searching_fix;
uint32_t tx_state = 0;
bool scenario; //indica si se ha escogido ya escenario
int times;	//para el parpadeo del led
int per_adv; //periodo de advertisement

bool timer_pressed = false; //indica si el timer que cuenta el tiempo que lleva el botón presionado está activo
int seconds = 0; //tiempo que lleva pulsado el botón

int t = 0;
bool on = false;

static struct nrf_modem_gnss_pvt_data_frame last_pvt;
struct TFix fix;
// ------------------------------------------------

// THREAD CONFIG PARAMETERS
// ------------------------------------------------
/* size of stack area used by each thread */
#define STACKSIZE 1024
/* scheduling priority used by each thread */
#define PRIORITY 7
K_THREAD_DEFINE(h, STACKSIZE, hebra, NULL, NULL, NULL, PRIORITY, 0, 0);
// ------------------------------------------------


void main(void)
{
	uint8_t buf[8];
	int err;
	uint32_t ev;
	int64_t unix_time_ns;
	struct timespec t;

	memset(buf, 0, sizeof(buf));

	//inicialización del logger
	log_init();
	LOG_INF("Starting BLE Advertisement Config");

	//Inicialización de los leds RGB
	if (leds_init() != 0) {
		LOG_ERR("Failed to initialize leds");
		while(1);
	}

	//Inicialización de la uart
	uart1 = DEVICE_DT_GET(DT_NODELABEL(uart1));
	__ASSERT(uart1, "Failed to get the device");

	//Establezco ISR de la UART
	uart_irq_callback_set(uart1, uart1_irq_handler);
	//Se habilitan las interrupciones
	uart_irq_rx_enable(uart1);

	//Inicialización GPS
	if (modem_init() != 0) {
		LOG_ERR("Failed to initialize modem");
		while(1);
	}

	err = gnss_init_and_start();
	if(err != 0) {
		LOG_ERR("Failed to initialize and start GNSS");
		while(1);
	}

	LOG_INF("Esperando a que se obtenga un fix válido");

	ev = k_event_wait(&event, fix_obtained, true, K_FOREVER); //espero hasta obtener información gps correcta
	if((ev & fix_obtained) != 0){
		//Parpadeo del led en rojo para indicar que se ha encontrado un fix válido
		blink_indication(true, false, false);

		err = clock_gettime(CLOCK_REALTIME, &t);
		if(err){
			LOG_ERR("clock_gettime error (%d)", err);
			while(1);
		}

		unix_time_ns = t.tv_sec * 1E9 + t.tv_nsec;
		LOG_INF("unix_time_ns: %lld", unix_time_ns);

		//Se envía la hora actual a través de la uart
		serializar(unix_time_ns, buf);
	
		//Inicialiación dk buttons
		err = dk_buttons_init(button_callback);
		if(err < 0){
			LOG_INF("(dk_buttons_init) error %d", err);
		}

	}
}


int leds_init(){
	int err;

	leds[RED] = device_get_binding(LED0);
	if (leds[RED] == NULL) {
		return -1;
	}

	leds[GREEN] = device_get_binding(LED1);
	if (leds[GREEN] == NULL) {
		return -1;
	}

	leds[BLUE] = device_get_binding(LED2);
	if (leds[BLUE] == NULL) {
		return -1;
	}

	err = gpio_pin_configure(leds[RED], PIN0, GPIO_OUTPUT_ACTIVE | FLAGS0);
	if (err < 0) {
		return -1;
	}

	err = gpio_pin_configure(leds[GREEN], PIN1, GPIO_OUTPUT_ACTIVE | FLAGS1);
	if (err < 0) {
		return -1;
	}

	err = gpio_pin_configure(leds[BLUE], PIN2, GPIO_OUTPUT_ACTIVE | FLAGS2);
	if (err < 0) {
		return -1;
	}

	gpio_pin_set(leds[RED], PIN0, true);
	gpio_pin_set(leds[GREEN], PIN1, true);
	gpio_pin_set(leds[BLUE], PIN2, true);

	return 0;
}


static int modem_init(void){

	if (strlen(CONFIG_GNSS_SAMPLE_AT_MAGPIO) > 0) {
		if (nrf_modem_at_printf("%s", CONFIG_GNSS_SAMPLE_AT_MAGPIO) != 0) {
			LOG_ERR("Failed to set MAGPIO configuration");
			return -1;
		}
	}

	if (strlen(CONFIG_GNSS_SAMPLE_AT_COEX0) > 0) {
		if (nrf_modem_at_printf("%s", CONFIG_GNSS_SAMPLE_AT_COEX0) != 0) {
			LOG_ERR("Failed to set COEX0 configuration");
			return -1;
		}
	}

	if (lte_lc_init() != 0) {
		LOG_ERR("Failed to initialize LTE link controller");
		return -1;
	}

	return 0;
}


static int gnss_init_and_start(void){
	int32_t err;
	uint16_t fix_retry;
	uint16_t fix_interval;

	/* Enable GNSS. */
	err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
	if( err != 0) {
		LOG_ERR("Failed to activate GNSS functional mode (%d)", err);
		return -1;
	}

	// Configure GNSS. 
	err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
	if(err != 0) {
		LOG_ERR("Failed to set GNSS event handler (%d)", err);
		return -1;
	}

	// This use case flag should always be set. 
	uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

	err = nrf_modem_gnss_use_case_set(use_case);
	if(err != 0) {
		LOG_WRN("Failed to set GNSS use case (%d)", err );
		return -1;
	}

	// Default to no power saving. 
	uint8_t power_mode = NRF_MODEM_GNSS_PSM_DISABLED;
	err = nrf_modem_gnss_power_mode_set(power_mode);
	if(err != 0) {
		LOG_ERR("Failed to set GNSS power saving mode (%d)", err);
		return -1;
	}

	// Single-fix: GNSS is allowed to run indefinitely until a valid PVT estimate is produced
	fix_retry = 0;
	fix_interval = 0;

	err = nrf_modem_gnss_fix_retry_set(fix_retry);
	if( err != 0) {
		LOG_ERR("Failed to set GNSS fix retry (%d)", err);
		return -1;
	}

	err = nrf_modem_gnss_fix_interval_set(fix_interval);
	if(err != 0) {
		LOG_ERR("Failed to set GNSS fix interval (%d)", err);
		return -1;
	}

	err = nrf_modem_gnss_start();
	if( err != 0) {
		LOG_ERR("Failed to start GNSS (%d)", err);
		return -1;
	}

	return 0;
}


static void gnss_event_handler(int e){
	struct tm date_time;
	int retval;
	int64_t unix_time_ms;
	
	LOG_INF("GNSS event handler, event %d", e);	
	switch (e) {
		//NRF_MODEM_GNSS_EVT_PVT es un evento que se activa siempre (cada 1 seg), haya un fix válido o no
		case NRF_MODEM_GNSS_EVT_PVT:
			retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
			LOG_INF("Flags: %02x", last_pvt.flags);
			print_satellite_stats(&last_pvt);

			if ((retval == 0) && (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)) {
				date_time.tm_sec = last_pvt.datetime.seconds; //aqui puede haber un problema porque va de 0-60
				date_time.tm_min = last_pvt.datetime.minute;
				date_time.tm_hour = (last_pvt.datetime.hour + 2)%24; //el gps da una hora 2 menos que la actual (UTC)
				date_time.tm_mday = last_pvt.datetime.day;
				date_time.tm_mon = last_pvt.datetime.month - 1; //porque va de [1, 12]
				date_time.tm_year = last_pvt.datetime.year - 1900; //resto al año actual 1900
				LOG_INF("%d, %d, %d, %d, %d, %d ", date_time.tm_sec, date_time.tm_min, date_time.tm_hour, date_time.tm_mday, date_time.tm_mon, date_time.tm_year);


				date_time_set(&date_time);
				unix_time_ms = (int64_t)timeutil_timegm64(&date_time) * 1000;
				LOG_INF("%lld", unix_time_ms);

				k_event_set(&event, fix_obtained);
			}
			
		break;
	}
}


static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
	uint8_t tracked   = 0;
	uint8_t in_fix    = 0;
	uint8_t unhealthy = 0;

	for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i) {
		if (pvt_data->sv[i].sv > 0) {
			tracked++;

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
				in_fix++;
			}

			if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY) {
				unhealthy++;
			}
		}
	}

	LOG_INF("Tracking: %2d Using: %2d Unhealthy: %d", tracked, in_fix, unhealthy);
}


void blink_callback(struct k_timer *timer){

	if(t < times){
		gpio_pin_set(leds[RED], PIN0, on);
		on = !on;
		t = t + 1;
	}else{
		k_timer_stop(timer);
		t = 0;
		on = false;
	}
}





void send_command(unsigned char m){
    uart_poll_out(uart1, m);
}


void serializar(int64_t time, uint8_t* ser){

	memcpy(ser, &time, sizeof(int64_t));
	
	for(int i=0; i < 8; i++){
		//LOG_INF("%02x ", ser[i]);
		uart_poll_out(uart1, ser[i]);
	}
}


void blink_indication(bool red, bool green, bool blue){
	int cont = 0;

	while(cont < 3){
		gpio_pin_set(leds[RED], PIN0, red);
		gpio_pin_set(leds[GREEN], PIN1, green);
		gpio_pin_set(leds[BLUE], PIN2, blue);

		k_sleep(K_MSEC(500));

		gpio_pin_set(leds[RED], PIN0, false);
		gpio_pin_set(leds[GREEN], PIN1, false);
		gpio_pin_set(leds[BLUE], PIN2, false);

		k_sleep(K_MSEC(500));
		cont++;
	}
}

void button_callback(uint32_t button_state, uint32_t has_changed)
{
	LOG_INF("Button callback");

	if (((has_changed & DK_BTN1_MSK) != 0) && (button_state))
	{
		// Se ha presionado el botón 1: puede ser para cambiar de escenario (2 seg) o para iniciar la prueba en el escenario seleccionado
		LOG_INF("Botón presionado");
		k_timer_start(&timer_button_pressed, K_MSEC(PER), K_MSEC(PER));

	}

	if (((has_changed & DK_BTN1_MSK) != 0) && (!button_state))
	{

		LOG_INF("Botón soltado");
		k_timer_stop(&timer_button_pressed);

		if (seconds >= 2)
		{
			LOG_INF("Reinicio first");
			send_command(set_first);
			gpio_pin_set(leds[RED], PIN0, false);
			gpio_pin_set(leds[GREEN], PIN1, false);
			gpio_pin_set(leds[BLUE], PIN2, false);
		}else{
			LOG_INF("Desbloqueo hebras");
			send_command(unblock_threads);
		}

		seconds = 0;
	}
}

static void uart1_irq_handler(const struct device *dev, void *context){
	int err;
	char c;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			err = uart_fifo_read(dev, &c, sizeof(char)); 
			if(err <= 0){
				LOG_ERR("(uart_fifo_read) error %d", err);
			}else{
				LOG_INF("Recibido %02x a través de la uart", c);
				if((c & first_received) != 0){
					k_sem_give(&first_sem);
				}
			}
		}
	}
}

void hebra(void){
	int err;

	while(1){
		err = k_sem_take(&first_sem, K_FOREVER);
		if(err == 0){
			gpio_pin_set(leds[RED], PIN0, true);
			gpio_pin_set(leds[GREEN], PIN1, false);
			gpio_pin_set(leds[BLUE], PIN2, true);
		}
	}
}

static void timer_callback(struct k_timer *timer){
	LOG_INF("Timeout!");
	seconds = seconds + 1;

	if(seconds == 2){
		gpio_pin_set(leds[RED], PIN0, false);
		gpio_pin_set(leds[GREEN], PIN1, false);
		gpio_pin_set(leds[BLUE], PIN2, true);
	}
}

