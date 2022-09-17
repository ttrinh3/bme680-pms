#include <stdio.h>
#include "esp8266_wrapper.h"
#include "bme68x.h"
#include "bme68x_defs.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"


#include "driver/i2c.h"
#include "esp_types.h"

//declarations
void bme680_i2c_init(struct bme68x_dev *dev);
void User_delay_us(uint32_t period, void *intf_ptr);
void BME68X_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void BME68X_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
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
    //init this first to allow the gpios to work
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
	dev->read  = BME68X_i2c_read; //this seems like functions that he create and points to
	dev->write = BME68X_i2c_write;//again, this is a self created function (OK)
	dev->delay_us = User_delay_us;//self created (defined in this file)
	//safe side -> SW-Reset
	const uint8_t com_res = BME68X_SOFT_RESET_CMD; //OK makes sense
	BME68X_i2c_write(BME68X_REG_SOFT_RESET, &com_res, 1, dev ); //OK makes sense
	User_delay_us(5*1000, NULL); // > 2ms PowerOn-Reset

	// return (bme68x_init(dev) << 1); //OK makes sense
}

void BME68X_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr){
    uint8_t dev_addr = *((uint8_t *)intf_ptr); //stays the same
    i2c_cmd_handle_t cmd_handle; 
    //tx_start_bit();
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle); //replaces above

    //tx_byte((dev_addr << 1) & 0xFE);
    i2c_master_write_byte(cmd_handle,(0x77 << 1) | I2C_MASTER_WRITE, true); //replaces above
    //uint8_t x = rx_bit(); what's the point of this line? probaly a ack or nack
    uint8_t x;
    //tx_byte(reg_addr);
    i2c_master_write_byte(cmd_handle,reg_addr, true);//replaces above
    // x |= rx_bit();//what's the point of this line
    for (int i = 0; i < len; i++) {
        // tx_byte(reg_data[i]);
    	i2c_master_write_byte(cmd_handle,reg_data[i], true);//replace the above
    	// x |= rx_bit(); probably an ack
    }
    // tx_stop_bit();
    i2c_master_stop(cmd_handle); //replaces the above
    i2c_master_cmd_begin(I2C_NUM_0,cmd_handle, 2000/ portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);
}

// period x cycles per us
void User_delay_us(uint32_t period, void *intf_ptr) {
	// ulp_riscv_delay_cycles(period * ULP_RISCV_CYCLES_PER_US);
    vTaskDelay((period*1)/(portTICK_RATE_MS*1000));
}

void BME68X_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
    uint8_t dev_addr = *((uint8_t *)intf_ptr); //stays the same
    BME68X_i2c_write(reg_addr, NULL, 0, &dev_addr); //stays the same
    i2c_cmd_handle_t cmd_handle; // create a handle for i2c ops
    // tx_start_bit();//replaced by below
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle); //replaces above
    // tx_byte((dev_addr << 1) | 0x01);//
    i2c_master_write_byte(cmd_handle,(0x77 << 1) | I2C_MASTER_WRITE, true); //replaces above AND below
    //since it is acking by setting the last parameter to true
    // x |= rx_bit();
    //read data sequentially
	uint8_t y;
	for (int j = 0; j < len; j++) {
		// for (int i = 0; i < 8; i++) {
		// 	y <<= 1;
		// 	y |= rx_bit();
		// } // don't need to read bit by bit, replaced by byte read below
        //only nack when it's the last one 
    //check if it's the last iteration
        if (j == (len-1)){
            i2c_master_read_byte(cmd_handle,(uint8_t *)&y, I2C_MASTER_NACK);
        }
        else{ //if not then keep acking and reading
            i2c_master_read_byte(cmd_handle,(uint8_t *)&y, I2C_MASTER_ACK);
        }
		reg_data[j] = y;
		// if (j < (len-1)) tx_ack_bit();// replaced by above ops
	}
	// tx_nack_bit();	//last readed byte -> acknowledge with NACK//i already took care of this, look above 
    // tx_stop_bit();
    i2c_master_stop(cmd_handle); //replace stop bit
    
    //error returns 0x107 (or 263) which means OPERATION TIMED OUT or bruh wtfff
    i2c_master_cmd_begin(I2C_NUM_0,cmd_handle, 2000/ portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);
}

