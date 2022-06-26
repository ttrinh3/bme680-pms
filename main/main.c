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


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "getSensorReadings.h"

void user_init(void)
{
  while(1){
    getSensorReadings();
    vTaskDelay(1000/portTICK_RATE_MS);

  }
}
