/* Active click feedback Example */
#include <driver/adc.h>
#include <driver/i2c.h>
#include <driver/mcpwm.h>
#include <driver/uart.h>
#include <esp_event_loop.h>
#include <esp_log.h>
#include <esp_spi_flash.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <lwip/dns.h>
#include <lwip/err.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <math.h>
#include <nvs_flash.h>
#include <rom/uart.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "SSCI_BME280.h"
#include "connection.h"

const char *TAG = "main";

//----------------------------------------------------------------

#define MAX_COUNT 1000

int ad_w = 0;

int count = 0;

//----------------------------------------------------------------
// I2C for BME280

#define I2C_PORT 1

#define I2C_SDA_IO 18
#define I2C_SCL_IO 19
#define I2C_FREQ_HZ 400000
#define I2C_TX_BUF_DISABLE 0
#define I2C_RX_BUF_DISABLE 0

SSCI_BME280 BME280;

#define BME280_OSRS_T 0b001
#define BME280_OSRS_P 0b001
#define BME280_OSRS_H 0b001
#define BME280_MODE 0b11
#define BME280_T_SB 0b000
#define BME280_FILTER 0b001
#define BME280_SPI3W_EN 0

static esp_err_t initialize_i2c_master(void) {
  i2c_config_t conf;

  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_FREQ_HZ;
  conf.clk_flags = 0;
  esp_err_t err = i2c_param_config(I2C_PORT, &conf);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "error on i2c_param_config()");
    return err;
  }

  err = i2c_driver_install(I2C_PORT, conf.mode, I2C_RX_BUF_DISABLE,
                           I2C_TX_BUF_DISABLE, 0);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "error on i2c_driver_install()");
    return err;
  }

  err = i2c_set_timeout(I2C_PORT, 0xFFFFF);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "error on i2c_set_timeout()");
    return err;
  }
  return ESP_OK;
}

esp_err_t initialize_BME280(void) {
  esp_err_t err;

  err = BME280.setMode(I2C_PORT, BME280_ADDRESS, BME280_OSRS_T, BME280_OSRS_P,
                       BME280_OSRS_H, BME280_MODE, BME280_T_SB, BME280_FILTER,
                       BME280_SPI3W_EN);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "error on setMode()");
    return err;
  }

  err = BME280.readTrim();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "error on readTrim()");
    return err;
  }

  return ESP_OK;
}

//----------------------------------------------------------------
/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
// #define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
// #define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

/* FreeRTOS event group to signal when we are connected & ready to make a
 * request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

//----------------------------------------------------------------
// WiFi Initialize

static esp_err_t event_handler(void *ctx, system_event_t *event) {
  //    ESP_LOGI(TAG, "event_handler.");
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
      //        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_START");
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      //        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_GOT_IP");
      xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      /* This is a workaround as ESP32 WiFi libs don't currently
         auto-reassociate. */
      //        ESP_LOGI(TAG, "event_handler. SYSTEM_EVENT_STA_DISCONNECTED");
      esp_wifi_connect();
      xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
      break;
    default:
      break;
  }
  return ESP_OK;
}

static void initialise_wifi(void) {
  wifi_event_group = xEventGroupCreate();

  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
#if 0
/* start static IP addr */
    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);

    tcpip_adapter_ip_info_t ipInfo;
	int sekiji=0;
    IP4_ADDR(&ipInfo.ip, 172,16,11,70+sekiji);
    IP4_ADDR(&ipInfo.gw, 172,16,11,251);
    IP4_ADDR(&ipInfo.netmask, 255,255,255,0);
    tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);

    ip_addr_t dnsserver;
    IP_ADDR4( &dnsserver, 172,16,11,251);
    dns_setserver(0, &dnsserver);
/* end static IP addr */
#endif
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  /* ------------------------*/
  //    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", WIFI_SSID);
  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  sprintf(reinterpret_cast<char *>(wifi_config.sta.ssid), WIFI_SSID);
  sprintf(reinterpret_cast<char *>(wifi_config.sta.password), WIFI_PASS);
  //    wifi_config_t wifi_config = { };
  //    wifi_config.sta.ssid=(char *) WIFI_SSID;
  //    wifi_config.sta.password=(char *) WIFI_PASS;
  /* ------------------------*/
  //    wifi_config_t wifi_config = {
  //        .sta = {
  //            { .ssid = WIFI_SSID },
  //            { .password = WIFI_PASS },
  ////            { .ssid = EXAMPLE_WIFI_SSID },
  ////            { .password = EXAMPLE_WIFI_PASS },
  //        },
  //    };
  /* ------------------------*/
  ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

//----------------------------------------------------------------
// Data Send and receive

static void http_get_task(void *pvParameters) {
  while (1) {
  }
}

//----------------------------------------------------------------
// #define USE_TIMER   //  Whther use the timer or not. Without this definition,
// the function is called from a normal task.

#ifdef USE_TIMER
#define DT \
  0.0001  //  In the case of the timer, the minimum period is 50 micro second.
#else
#define DT (1.0 / configTICK_RATE_HZ)
//  In the case of the task, the time period is the time slice of the OS
//  specified in menuconfig, which is set to 1 ms=1 kHz.
#endif

void readTempFunc(void *arg) {
  double temp, press, hum;
  esp_err_t err = BME280.readData(&temp, &press, &hum);
  ESP_ERROR_CHECK(err);
  printf(
      "tempreture: %lf\n"
      "pressure: %lf\n"
      "humidity: %lf\n",
      temp, press, hum);
  sleep(3);
}

#ifndef USE_TIMER
void readTempTask(void *arg) {
  esp_err_t err = initialize_BME280();
  ESP_ERROR_CHECK(err);
  while (1) {
    readTempFunc(arg);
    vTaskDelay(1);
  }
}
#endif

extern "C" void app_main()
// void app_main()
{
  //----------------------------------
  // Initialize NVS
  ESP_LOGI("main", "Initialize NVS");
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  //----------------------------------

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
         (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
  printf("silicon revision %d, ", chip_info.revision);
  printf(
      "%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
      (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  //----------------------------------
  ESP_LOGI("main", "Initialize I2C");
  ret = initialize_i2c_master();
  ESP_ERROR_CHECK(ret);

  //----------------------------------
  ESP_LOGI("main", "Initialize WiFi");
  initialise_wifi();
  //----------------------------------
  printf("!!! Active Haptic Feedback Start !!!\n");

  ESP_LOGI("main", "Initialize ADC");
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  ESP_LOGI("main", "Initialize PWM");
  // 1. mcpwm gpio initialization
  const int GPIO_PWM0A_OUT = 16;
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
  // 2. initial mcpwm configuration
  printf("Configuring Initial Parameters of mcpwm...\n");
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;  // frequency = 1000Hz,
  pwm_config.cmpr_a = 0;        // duty cycle of PWMxA = 0
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0,
             &pwm_config);  // Configure PWM0A with above settings
  mcpwm_set_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0, 20000);
  gpio_config_t conf;
  conf.pin_bit_mask = (1 << (17)) | (1 << (5));
  conf.mode = GPIO_MODE_OUTPUT;
  conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  conf.pull_up_en = GPIO_PULLUP_DISABLE;
  conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&conf);

#ifdef USE_TIMER
  esp_timer_init();
  esp_timer_create_args_t timerDesc = {
    callback : hapticFunc,
    arg : NULL,
    dispatch_method : ESP_TIMER_TASK,
    name : "haptic"
  };
  esp_timer_handle_t timerHandle = NULL;
  esp_timer_create(&timerDesc, &timerHandle);
  esp_timer_start_periodic(
      timerHandle,
      (int)(1000 * 1000 * DT));  // period in micro second (100uS=10kHz)
#else
  TaskHandle_t taskHandle = NULL;
  xTaskCreate(readTempTask, "Temp", 1024 * 15, NULL, 6, &taskHandle);
#endif
  xTaskCreate(&http_get_task, "http_get_task", 1024 * 15, NULL, 3, NULL);

  uart_driver_install(UART_NUM_0, 1024, 1024, 10, NULL, 0);
  while (1) {
    uint8_t ch;
    uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY);
    printf("'%c' received.\r\n", ch);
    switch (ch) {
      case 'a':
        //  do something
        break;
    }
  }
}