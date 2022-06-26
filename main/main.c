#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "connect.h"
#include "mqtt_client.h"
#include "bme680.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pms5003.h"

#define I2C_BUS       0
#define I2C_SCL_PIN   14
#define I2C_SDA_PIN   13
#define I2C_FREQ      I2C_FREQ_100K

/* -- user tasks --------------------------------------------------- */

static bme680_sensor_t* sensor = 0;

#define TAG "MQTT"

xQueueHandle readingQueue;
TaskHandle_t taskHandle;

const uint32_t WIFI_CONNEECTED = BIT1;
const uint32_t MQTT_CONNECTED = BIT2;
const uint32_t MQTT_PUBLISHED = BIT3;

void mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
  switch (event->event_id)
  {
  case MQTT_EVENT_CONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
    xTaskNotify(taskHandle, MQTT_CONNECTED, eSetValueWithOverwrite);
    break;
  case MQTT_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    break;
  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_UNSUBSCRIBED:
    ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
    break;
  case MQTT_EVENT_PUBLISHED:
    ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
    xTaskNotify(taskHandle, MQTT_PUBLISHED, eSetValueWithOverwrite);
    break;
  case MQTT_EVENT_DATA:
    ESP_LOGI(TAG, "MQTT_EVENT_DATA");
    printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
    printf("DATA=%.*s\r\n", event->data_len, event->data);
    break;
  case MQTT_EVENT_ERROR:
    ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
    break;
  default:
    ESP_LOGI(TAG, "Other event id:%d", event->event_id);
    break;
  }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
  mqtt_event_handler_cb(event_data);
}

void MQTTLogic(int sensorReading)
{
  uint32_t command = 0;
  esp_mqtt_client_config_t mqttConfig = {
      .uri = "mqtt://mqtt.eclipseprojects.io"};
  esp_mqtt_client_handle_t client = NULL;

  while (true)
  {
    xTaskNotifyWait(0, 0, &command, portMAX_DELAY);
    switch (command)
    {
    case WIFI_CONNEECTED:
      client = esp_mqtt_client_init(&mqttConfig);
      esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
      esp_mqtt_client_start(client);
      break;
    case MQTT_CONNECTED:
      esp_mqtt_client_subscribe(client, "/topic/my/subscription/1", 2);
      char data[50];
      sprintf(data, "%d", sensorReading);
      printf("sending data: %d", sensorReading);




       pms5003_config_t pms0 = {
        .set_pin = GPIO_NUM_13,
        .reset_pin = GPIO_NUM_27,
        .mode_pin = GPIO_NUM_26,
        .rxd_pin = GPIO_NUM_11,
        .txd_pin = GPIO_NUM_12,
        .uart_instance = UART_NUM_1,
        .uart_buffer_size = 128
    };
    pms5003_setup(&pms0);
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);


    /** -- MANDATORY PART -- */

    #ifdef SPI_USED

    spi_bus_init (SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);

    // init the sensor connected to SPI_BUS with SPI_CS_GPIO as chip select.
    sensor = bme680_init_sensor (SPI_BUS, 0, SPI_CS_GPIO);

    #else  // I2C

    // Init all I2C bus interfaces at which BME680 sensors are connected
    i2c_init(I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);

    // init the sensor with slave address BME680_I2C_ADDRESS_2 connected to I2C_BUS.
    sensor = bme680_init_sensor (I2C_BUS, BME680_I2C_ADDRESS_2, 0);

    #endif  // SPI_USED

    if (sensor)
    {
        /** -- SENSOR CONFIGURATION PART (optional) --- */

        // Changes the oversampling rates to 4x oversampling for temperature
        // and 2x oversampling for humidity. Pressure measurement is skipped.
        bme680_set_oversampling_rates(sensor, osr_4x, osr_none, osr_2x);

        // Change the IIR filter size for temperature and pressure to 7.
        bme680_set_filter_size(sensor, iir_size_7);

        // Change the heater profile 0 to 200 degree Celcius for 100 ms.
        bme680_set_heater_profile (sensor, 0, 200, 100);
        bme680_use_heater_profile (sensor, 0);

        // Set ambient temperature to 10 degree Celsius
        bme680_set_ambient_temperature (sensor, 10);

        /** -- TASK CREATION PART --- */

        // must be done last to avoid concurrency situations with the sensor 
        // configuration part

        // Create a task that uses the sensor
        //start while loop here
    bme680_values_float_t values;

    TickType_t last_wakeup = xTaskGetTickCount();

    // as long as sensor configuration isn't changed, duration is constant
    uint32_t duration = bme680_get_measurement_duration(sensor);

    pms5003_measurement_t reading;
    while (1)
    {
        // trigger the sensor to start one TPHG measurement cycle
        if (bme680_force_measurement (sensor))
        {
            // passive waiting until measurement results are available
            vTaskDelay (duration);

            // alternatively: busy waiting until measurement results are available
            // while (bme680_is_measuring (sensor)) ;

            // get the results and do something with them
            if (bme680_get_results_float (sensor, &values)){
              char buffer[600];
              char buffer2[1000];
                sprintf(buffer," BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                       values.temperature, values.humidity,
                       values.pressure, values.gas_resistance);
                        pms5003_make_measurement(&pms0, &reading);
                        pms5003_print_measurement(&reading);
                        sprintf(buffer2," PM1.0: %d,PM2.5: %d, PM10: %d %s", reading.pm1_0_std, reading.pm2_5_std, reading.pm10_std,buffer);

esp_mqtt_client_publish(client, "topic/my/timothy/1", buffer2, strlen(buffer2), 2, false);
                       }
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }
    }
    else
        printf("Could not initialize BME680 sensor\n");

      
      break;
    case MQTT_PUBLISHED:
      esp_mqtt_client_stop(client);
      esp_mqtt_client_destroy(client);
      esp_wifi_stop();
      return;
    default:
      break;
    }
  }
}

void OnConnected(void *para)
{

  while (true)
  {
    int sensorReading;
    if (xQueueReceive(readingQueue, &sensorReading, portMAX_DELAY))
    {
      ESP_ERROR_CHECK(esp_wifi_start());
      MQTTLogic(sensorReading);
    }
  }
}

int test_variable = 0;
void generateReading(void *params)
{
  while (true)
  {
    // int random = esp_random();
    int random = (test_variable++);
    xQueueSend(readingQueue, &random, 2000 / portTICK_PERIOD_MS);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}





/**
 * Simple example with one sensor connected either to I2C bus 0 or
 * SPI bus 1.
 *
 * Harware configuration:
 *
 *   I2C
 *
 *   +-----------------+   +----------+
 *   | ESP8266 / ESP32 |   | BME680   |
 *   |                 |   |          |
 *   |   GPIO 14 (SCL) ----> SCL      |
 *   |   GPIO 13 (SDA) <---> SDA      |
 *   +-----------------+   +----------+
 *
 *   SPI   
 *
 *   +-----------------+   +----------+      +-----------------+   +----------+
 *   | ESP8266         |   | BME680   |      | ESP32           |   | BME680   |
 *   |                 |   |          |      |                 |   |          |
 *   |   GPIO 14 (SCK) ----> SCK      |      |   GPIO 16 (SCK) ----> SCK      |
 *   |   GPIO 13 (MOSI)----> SDI      |      |   GPIO 17 (MOSI)----> SDI      |
 *   |   GPIO 12 (MISO)<---- SDO      |      |   GPIO 18 (MISO)<---- SDO      |
 *   |   GPIO 2  (CS)  ----> CS       |      |   GPIO 19 (CS)  ----> CS       |
 *   +-----------------+    +---------+      +-----------------+   +----------+
 */

/* -- use following constants to define the example mode ----------- */

// #define SPI_USED

/* -- includes ----------------------------------------------------- */


/* -- platform dependent definitions ------------------------------- */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 4096

// SPI interface definitions for ESP32
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO  16
#define SPI_MOSI_GPIO 17
#define SPI_MISO_GPIO 18
#define SPI_CS_GPIO   19

#else  // ESP8266 (esp-open-rtos)

// user task stack depth for ESP8266
#define TASK_STACK_DEPTH 2048

// SPI interface definitions for ESP8266
#define SPI_BUS       1
#define SPI_SCK_GPIO  14
#define SPI_MOSI_GPIO 13
#define SPI_MISO_GPIO 12
#define SPI_CS_GPIO   2   // GPIO 15, the default CS of SPI bus 1, can't be used

#endif  // ESP_PLATFORM

// I2C interface defintions for ESP32 and ESP8266


/*
 * User task that triggers measurements of sensor every seconds. It uses
 * function *vTaskDelay* to wait for measurement results. Busy wating
 * alternative is shown in comments
 */


/* -- main program ------------------------------------------------- */

void user_init(void)
{
   readingQueue = xQueueCreate(sizeof(int), 10);
  wifiInit();
  gettimeofday();
  xTaskCreate(OnConnected, "handel comms", 1024 * 5, NULL, 5, &taskHandle);
  xTaskCreate(generateReading, "handel comms", 1024 * 5, NULL, 5, NULL);
  
    
// while (1){


//   printf("%lld\n",esp_timer_get_time());
//   vTaskDelay(1000/portTICK_RATE_MS)
// }
}

