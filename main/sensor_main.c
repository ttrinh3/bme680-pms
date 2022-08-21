#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/ulp.h"
#include "soc/rtc.h"
#include "esp32s2/ulp_riscv.h"

#include "ulp_main.h"  				//ulp_xxx.h automatically generated

//wakeup-interval ulp
#define INTERVAL_MS			5000

//Main-wakeup conditions
#define TEMP_THRESHOLD		0.25	//delta °C
#define HUMI_THRESHOLD		2		//delta %
#define PRES_THRESHOLD		2		//delta hPa
#define MAX_FORCE_REPORT	0		//at least after x ulp-loops / 0 => every time

//individual RTC-IO 1..21
#define I2C_MASTER_SDA_IO 	8
#define I2C_MASTER_SCL_IO 	9


//Definitions ULP-Programm
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");


void init_ulp() {
	//ULP-Clockspeed
	uint32_t rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 100);
	if (rtc_8md256_period > 0) {	//nach esp_restart() => 0
		uint32_t rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;
		printf("RTC FastFreq: %dHz\n", rtc_fast_freq_hz);
	}
   	printf("Initializing ULP\n");

   	//ULP-Programm load
    esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

   	printf("ULP-Program-Size: %d\n", ulp_main_bin_end - ulp_main_bin_start);

   	//ULP-Programm starten
   	ESP_ERROR_CHECK(ulp_riscv_run());
}

// --------------------------------------------------------------------------------------------------------------

// Wakeup-Stub - runs after wakeup before boot
void RTC_IRAM_ATTR esp_wake_deep_sleep(void) {
 	esp_default_wake_deep_sleep();
 	//...
}

void app_main(void) {

	bool waked_up __attribute__((unused)) = false;
	esp_sleep_wakeup_cause_t rst_reason = esp_sleep_get_wakeup_cause();

	printf("\nRST-Cause: %d\n", rst_reason);

	if ((rst_reason != ESP_SLEEP_WAKEUP_ULP)  &&
    	(rst_reason != ESP_SLEEP_WAKEUP_EXT1) &&
		(rst_reason != ESP_SLEEP_WAKEUP_TIMER)) {

    	init_ulp();	//ULP-Programm load

    	//main wakeup-conditions
    	ulp_set_bme280_force_wake = MAX_FORCE_REPORT;
    	ulp_set_bme280_thres_temp = TEMP_THRESHOLD * 100;
    	ulp_set_bme280_thres_humi = HUMI_THRESHOLD * 1000;
    	ulp_set_bme280_thres_pres = PRES_THRESHOLD * 100;
    	ulp_bme280_sda = I2C_MASTER_SDA_IO;
    	ulp_bme280_scl = I2C_MASTER_SCL_IO;

	} else {
    	//from DeepSleep
    	waked_up = true;
	}

	if (ulp_bme280_status == 0) {	//no error in last ulp-run
		if (waked_up) {
			printf("Chip ID  : 0x%.2x\n", ulp_bme280_chip_id);
			printf("t-acquis : %.dms\n", ulp_bme280_acquisition_time_ms);
			printf("ulp-cycle: %d\n", ulp_bme280_cycles);
			printf("Press: %.2fhPa | Temp: %.2f°C | Humi: %.2f%%\n",
					ulp_bme280_pressure/100.0, (int32_t)ulp_bme280_temperature/100.0, ulp_bme280_humidity/1000.0);
		}
	} else {
		printf ("Fehler BME280: %d\n", ulp_bme280_status);
		printf("Chip ID: 0x%.2x\n",	ulp_bme280_chip_id);
	}

	//reduce sleep-current
	//do not use on inputs ! (increases current !)
//	rtc_gpio_isolate(GPIO_NUM_15);

	esp_deep_sleep_disable_rom_logging();
    ulp_set_wakeup_period(0, INTERVAL_MS *1000);  //ulp starten nach x ms
	esp_sleep_enable_ulp_wakeup();
   	printf("Goto DeepSleep for %.3fs\n", INTERVAL_MS / 1000.0);

	esp_deep_sleep_start();
}


