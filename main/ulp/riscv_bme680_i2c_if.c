/*
 * bme680_i2c_if.c
 *
 *  Created on: Timothy
 *  Author    : Timothy
 *	License   : Timothy
 *
 *  use actual driver from:
 *  Timothy
 *
*/

//relative Paths not found in Eclipse / IDF is ok
//probably doesn't need to be changed for bme680
//because they are ulp-specific
#include "ulp_riscv/ulp_riscv.h"
#include "ulp_riscv/ulp_riscv_utils.h"
#include "ulp_riscv/ulp_riscv_gpio.h"

//bme680 equivalents
//https://github.com/BoschSensortec/BME68x-Sensor-API
#include "BME680_driver-master/bme68x.h"
#include "BME680_driver-master/bme68x_defs.h"
/*
 * #define BME280_32BIT_ENABLE	!!! set in bme280_defs.h !!!
 */
//above is not applicable to bme680

//internal varibales
static uint8_t dev_addr;
static uint8_t first_run = 0;
//OK: properly linked
static struct bme68x_data bme68x_comp_data; //line 718 of bme68x.defs.h
//cache wakeup check
//no need for changes
static uint32_t mcycle = 0;
static uint32_t mtemp = 0;
static uint32_t mhumi = 0;
static uint32_t mpres = 0;
static uint32_t mgas = 0;


//external variables, access from main => ulp_xxxxx
//results to main
//author means that these variables read by main
uint32_t bme680_humidity;
uint32_t bme680_temperature;
uint32_t bme680_pressure;
uint32_t bme680_gas;//Added
uint32_t bme680_status;
uint32_t bme680_chip_id = 0;
uint32_t bme680_acquisition_time_ms;
uint32_t bme680_cycles = 0;
//set individual from main
//these are supposedly set from main, but I don't see the connection
//line 73? seems to be 1:1 match
uint32_t bme680_sda = 0;
uint32_t bme680_scl = 0;
uint32_t set_bme680_force_wake = 0;
uint32_t set_bme680_thres_temp = 0;
uint32_t set_bme680_thres_humi = 0;
uint32_t set_bme680_thres_pres = 0;
uint32_t set_bme680_thres_gas = 0; //don't forget to add this


// HW - Initialisierung --------------------------------------------

static void init_gpio() {
	// Setup GPIO für bitweise I2C
	//no need for change since these are not sensor-specific
    ulp_riscv_gpio_init(bme680_sda);
    ulp_riscv_gpio_input_enable(bme680_sda);
    ulp_riscv_gpio_set_output_mode(bme680_sda, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(bme680_sda);
    ulp_riscv_gpio_pulldown_disable(bme680_sda);
    ulp_riscv_gpio_output_level(bme680_sda, 0);

    ulp_riscv_gpio_init(bme680_scl);
    ulp_riscv_gpio_input_enable(bme680_scl);
    ulp_riscv_gpio_set_output_mode(bme680_scl, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(bme680_scl);
    ulp_riscv_gpio_pulldown_disable(bme680_scl);
    ulp_riscv_gpio_output_level(bme680_scl, 0);
}


// I2C - Bit & Byte - level ----------------------------------------
//Grundstellung High -> Input (ext. PullUp) / aktiv Low => Input & Output (fix low)
//no need to change because they are not sensor specific
#define SCL_L		ulp_riscv_gpio_output_enable(bme680_scl)
#define SCL_H		ulp_riscv_gpio_output_disable(bme680_scl)
#define X_SCL		ulp_riscv_gpio_get_level(bme680_scl)
#define SDA_L		ulp_riscv_gpio_output_enable(bme680_sda)
#define SDA_H		ulp_riscv_gpio_output_disable(bme680_sda)
#define X_SDA		ulp_riscv_gpio_get_level(bme680_sda)

#define CLK			20 	//ca. 20kHz Takt (measured)
#define T25			ulp_riscv_delay_cycles(CLK / 4)

//this is a sequence of bit manipulating functions that the author just defined above
static void tx_start_bit() {
	SDA_H; T25; SCL_H; T25; SDA_L; T25;	// -> SDA_L
}

static void tx_stop_bit() {	//SCL_H
	T25; SCL_L; SDA_L; T25; SCL_H; T25; SDA_H;
}

static void tx_1_bit() {	//SCL H
	T25; SCL_L; T25; SDA_H; T25; SCL_H; T25;
}

static void tx_0_bit() {	//SCL H
	T25; SCL_L; T25; SDA_L; T25; SCL_H; T25;	// -> SDA_L
}

static uint8_t rx_bit() {
	uint8_t x;
	T25;  SCL_L; SDA_H; T25; x = X_SDA; SCL_H;  T25;
	return x;
}

static void tx_ack_bit() {
	T25; SCL_L; T25; SDA_L; T25; SCL_H; T25;	// ->SDA_L
}

static void tx_nack_bit() {
	T25; SCL_L; T25; SDA_H; T25; SCL_H; T25;
}
//sends a byte probably because of the 8 in the for loop
static void tx_byte (uint8_t x) {
	for (int i = 0; i < 8; i++) {
		if ((x & 0x80) == 0) tx_0_bit();
		else tx_1_bit();
		x = x << 1;
	}
}


// I2C - adjustment to BME680.h --------------------------------
void user_delay_us(uint32_t period, void *intf_ptr) {
	ulp_riscv_delay_cycles(period * ULP_RISCV_CYCLES_PER_US);
}
//not directly affected but pay attention to the parameters passed in
int8_t bme680_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *((uint8_t *)intf_ptr);

    tx_start_bit();
    tx_byte((dev_addr << 1) & 0xFE);
    uint8_t x = rx_bit();
    tx_byte(reg_addr);
    x |= rx_bit();
    for (int i = 0; i < len; i++) {
    	tx_byte(reg_data[i]);
    	x |= rx_bit();
    }
    tx_stop_bit();
    return x;
}


int8_t bme680_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    //send source-addr to slave, without data
    uint8_t x = bme680_i2c_write(reg_addr, NULL, 0, &dev_addr);
    //sens slave-adr senden for preface reading process
    tx_start_bit();
    tx_byte((dev_addr << 1) | 0x01);
    x |= rx_bit();
    //read data sequentially
	uint8_t y;
	for (int j = 0; j < len; j++) {
		for (int i = 0; i < 8; i++) {
			y <<= 1;
			y |= rx_bit();
		}
		reg_data[j] = y;
		if (j < (len-1)) tx_ack_bit();
	}
	tx_nack_bit();	//last readed byte -> acknowledge with NACK
    tx_stop_bit();
    return x;/
}


// --------------------------------------------------------------------------------------------

//initialise chip after coldstart or device error
int8_t bme680_i2c_init(struct bme68x_dev *dev) {
	dev_addr = BME68X_I2C_ADDR_LOW;	//SDO 10k Pulldown on Sensorboard => addr = 0x76
	//for the above, the 680 and 280 have the exact same address
	//instead of primary and secondary in the 280, 
	//the 680 has the low and high, but values are the same. 
	dev->intf_ptr = &dev_addr; //OK makes sense
	dev->intf  = BME68X_I2C_INTF; //OK makes sense
	dev->read  = bme680_i2c_read; //this seems like functions that he create and points to
	dev->write = bme680_i2c_write;//again, this is a self created function (OK)
	dev->delay_us = user_delay_us;//self created (defined in this file)
	//safe side -> SW-Reset
	const uint8_t com_res = BME68X_SOFT_RESET_CMD; //OK makes sense
	bme280_i2c_write(BME68X_REG_SOFT_RESET, &com_res, 1, dev); //OK makes sense
	user_delay_us(5*1000, NULL); // > 2ms PowerOn-Reset

	return (bme68x_init(dev) << 1); //OK makes sense
}



//perform measurement an readinfg sensor-data
//duration about 70ms
int8_t stream_sensor_data_forced_mode(struct bme68x_dev *dev, struct bme68x_conf *dev2) {
	//inividaul setting, refer BME280-doc
	//the bme680 does not have settings lumped in with the dev struct
	//like the 280, it has another struct for that (bme68x_conf (dev2)), which I have added
	dev2->os_hum = BME68X_OS_2X; //OK make sense
	dev2->os_pres = BME68X_OS_8X;//OK make sense
	dev2->os_temp = BME68X_OS_2X;//OK make sense
	//even for the bme680, there is no gas OS. that's why it's missing
	dev2->filter = BME68X_FILTER_OFF;//OK make sense

	//NO 680 equivalent
	uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	//set oversampling und filter
	//NO 680 equivalent
	int8_t rslt = bme280_set_sensor_settings(settings_sel, dev);
	//NO 680 equivalent
	uint32_t req_delay_ms = bme280_cal_meas_delay(&dev->settings);
	bme280_acquisition_time_ms = req_delay_ms;
	//initiate measurement in force-mode (inividual)
	// rslt |= bme280_set_sensor_mode(BME68X_FORCED_MODE, dev);
	//Seems to be the proper replacement
	bme68x_set_op_mode(BME68X_FORCED_MODE, dev); //dev is the non-settings one, so it's correct
	//wait for complete plus safty
	dev->delay_us((req_delay_ms + 5) * 1000, dev->intf_ptr); //Possibly OK

	rslt |= bme68x_get_data(BME280_ALL, &bme280_comp_data, dev);
	rslt |= bme68x_set_op_mode(BME280_SLEEP_MODE, dev);

	return (rslt << 2);
}


int main (void) {
	bme680_cycles++;
	init_gpio();
	int8_t rslt = 0;
	struct bme280_dev bme280_device;

	if (first_run == 0) {
		rslt = bme280_i2c_init(&bme280_device);
		first_run = 1;
	}
	//Chip-ID zur Info
	uint8_t chip_id = 0;
	rslt |= bme280_get_regs(BME280_CHIP_ID_ADDR, &chip_id, 1, &bme280_device);
	bme280_chip_id = chip_id;

	rslt |= stream_sensor_data_forced_mode(&bme280_device);

	bme280_pressure = bme280_comp_data.pressure;			// pa
	bme280_temperature = bme280_comp_data.temperature;		// °C * 100
	bme280_humidity = bme280_comp_data.humidity;			// %  * 1000
	if (bme280_humidity == 0) rslt |= 1 << 3;				// check plausibility
	bme280_status = rslt;									// 0 == Ok
	if (rslt != 0) first_run = 0; //force reset

    //test wakeup conditions
    if ((++mcycle >= set_bme280_force_wake) ||							//max cycles since last main-wakeup
    	(abs(mtemp - bme280_temperature) >= set_bme280_thres_temp) ||	//delta temp
		(abs(mhumi - bme280_humidity) >= set_bme280_thres_humi) || 		//delta humi
		(abs(mpres - bme280_pressure) >= set_bme280_thres_pres)) {		//delta pres
    		mtemp = bme280_temperature;
    		mhumi = bme280_humidity;
    		mpres = bme280_pressure;
    		mcycle = 0;
    	ulp_riscv_wakeup_main_processor();
    }
}
