#include <stdio.h>
#include "esp8266_wrapper.h"
#include "bme68x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

//declarations
void bme680_i2c_init(struct bme68x_dev *dev);
void user_delay_us(uint32_t period, void *intf_ptr);
void app_main(void)
{
    static uint8_t dev_addr;
    int16_t temperature;
    uint32_t pressure;
    uint32_t humidity;
    uint32_t gas_resistance;
    struct bme68x_dev bme;
    struct bme68x_conf conf;
    struct bme68x_data data;
    uint8_t n_fields;
    i2c_init (0, 9, 8, 100000); //8 is sda, 9 is scl, 0 is i2c bus, last is frequency

    bme680_i2c_init(&bme);
    bme68x_init(&bme);
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_1X;
    bme68x_set_conf(&conf, &bme);
    struct bme68x_heatr_conf heatr_conf;
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);

    while (1){
        bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
        // ulp_riscv_delay_cycles(10000 * 17.5);
        uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
        bme.delay_us(del_period, bme.intf_ptr);
        bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
        vTaskDelay(2000/portTICK_RATE_MS);
        printf(" %d, %lu, %lu, %lu\n",
                        (data.temperature / 100),
                        (long unsigned int)data.pressure,
                        (long unsigned int)(data.humidity / 1000),
                        (long unsigned int)data.gas_resistance);

    }
}

void bme680_i2c_init(struct bme68x_dev *dev) {
	static uint8_t dev_addr = BME68X_I2C_ADDR_HIGH;	//SDO 10k Pulldown on Sensorboard => addr = 0x76
	//for the above, the 680 and 280 have the exact same address
	//instead of primary and secondary in the 280, 
	//the 680 has the low and high, but values are the same.
	//here he loads the dev with self written functions 
	dev->intf_ptr = &dev_addr; //OK makes sense
	dev->intf  = BME68X_I2C_INTF; //OK makes sense
	dev->read  = bme68x_i2c_read; //this seems like functions that he create and points to
	dev->write = bme68x_i2c_write;//again, this is a self created function (OK)
	dev->delay_us = user_delay_us;//self created (defined in this file)
	//safe side -> SW-Reset
	const uint8_t com_res = BME68X_SOFT_RESET_CMD; //OK makes sense
	i2c_slave_write(BME68X_REG_SOFT_RESET, &com_res, 1, dev ); //OK makes sense
	user_delay_us(5*1000, NULL); // > 2ms PowerOn-Reset

	// return (bme68x_init(dev) << 1); //OK makes sense
}

// period x cycles per us
void user_delay_us(uint32_t period, void *intf_ptr) {
	// ulp_riscv_delay_cycles(period * ULP_RISCV_CYCLES_PER_US);
    vTaskDelay((period*1)/(portTICK_RATE_MS*1000));
}

