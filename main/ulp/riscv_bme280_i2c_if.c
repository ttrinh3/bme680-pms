/*
 * bme280_i2c_if.c
 *
 *  Created on: 2021-11-14
 *  Author    : Joerg / DL7VMD
 *	License   : MIT
 *
 *  use actual driver from:
 *  https://github.com/BoschSensortec/BME280_driver
 *
*/

//relative Paths not found in Eclipse / IDF is ok
#include "ulp_riscv/ulp_riscv.h"
#include "ulp_riscv/ulp_riscv_utils.h"
#include "ulp_riscv/ulp_riscv_gpio.h"

#include "BME280_driver-master/bme280.h"
#include "BME280_driver-master/bme280_defs.h"

/*
 * #define BME280_32BIT_ENABLE	!!! set in bme280_defs.h !!!
 */


//internal varibales
static uint8_t dev_addr;
static uint8_t first_run = 0;
static struct bme280_data bme280_comp_data;
//cache wakeup check
static uint32_t mcycle = 0;
static uint32_t mtemp = 0;
static uint32_t mhumi = 0;
static uint32_t mpres = 0;


//external variables, access from main => ulp_xxxxx
//results to main
uint32_t bme280_humidity;
uint32_t bme280_temperature;
uint32_t bme280_pressure;
uint32_t bme280_status;
uint32_t bme280_chip_id = 0;
uint32_t bme280_acquisition_time_ms;
uint32_t bme280_cycles = 0;
//set individual from main
uint32_t bme280_sda = 0;
uint32_t bme280_scl = 0;
uint32_t set_bme280_force_wake = 0;
uint32_t set_bme280_thres_temp = 0;
uint32_t set_bme280_thres_humi = 0;
uint32_t set_bme280_thres_pres = 0;


// HW - Initialisierung --------------------------------------------

static void init_gpio() {
	// Setup GPIO für bitweise I2C
    ulp_riscv_gpio_init(bme280_sda);
    ulp_riscv_gpio_input_enable(bme280_sda);
    ulp_riscv_gpio_set_output_mode(bme280_sda, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(bme280_sda);
    ulp_riscv_gpio_pulldown_disable(bme280_sda);
    ulp_riscv_gpio_output_level(bme280_sda, 0);

    ulp_riscv_gpio_init(bme280_scl);
    ulp_riscv_gpio_input_enable(bme280_scl);
    ulp_riscv_gpio_set_output_mode(bme280_scl, RTCIO_MODE_OUTPUT_OD);
    ulp_riscv_gpio_pullup_disable(bme280_scl);
    ulp_riscv_gpio_pulldown_disable(bme280_scl);
    ulp_riscv_gpio_output_level(bme280_scl, 0);
}


// I2C - Bit & Byte - level ----------------------------------------
//Grundstellung High -> Input (ext. PullUp) / aktiv Low => Input & Output (fix low)
#define SCL_L		ulp_riscv_gpio_output_enable(bme280_scl)
#define SCL_H		ulp_riscv_gpio_output_disable(bme280_scl)
#define X_SCL		ulp_riscv_gpio_get_level(bme280_scl)
#define SDA_L		ulp_riscv_gpio_output_enable(bme280_sda)
#define SDA_H		ulp_riscv_gpio_output_disable(bme280_sda)
#define X_SDA		ulp_riscv_gpio_get_level(bme280_sda)

#define CLK			20 	//ca. 20kHz Takt (measured)
#define T25			ulp_riscv_delay_cycles(CLK / 4)

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

static void tx_byte (uint8_t x) {
	for (int i = 0; i < 8; i++) {
		if ((x & 0x80) == 0) tx_0_bit();
		else tx_1_bit();
		x = x << 1;
	}
}


// I2C - adjustment to BME280.h --------------------------------
void user_delay_us(uint32_t period, void *intf_ptr) {
	ulp_riscv_delay_cycles(period * ULP_RISCV_CYCLES_PER_US);
}

int8_t bme280_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
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


int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    uint8_t dev_addr = *((uint8_t *)intf_ptr);
    //send source-addr to slave, without data
    uint8_t x = bme280_i2c_write(reg_addr, NULL, 0, &dev_addr);
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
    return x;
}


// --------------------------------------------------------------------------------------------

//initialise chip after coldstart or device error
int8_t bme280_i2c_init(struct bme280_dev *dev) {
	dev_addr = BME280_I2C_ADDR_PRIM;	//SDO 10k Pulldown on Sensorboard => addr = 0x76
	dev->intf_ptr = &dev_addr;
	dev->intf  = BME280_I2C_INTF;
	dev->read  = bme280_i2c_read;
	dev->write = bme280_i2c_write;
	dev->delay_us = user_delay_us;
	//safe side -> SW-Reset
	const uint8_t com_res = BME280_SOFT_RESET_COMMAND;
	bme280_i2c_write(BME280_RESET_ADDR, &com_res, 1, dev);
	user_delay_us(5*1000, NULL); // > 2ms PowerOn-Reset

	return (bme280_init(dev) << 1);
}



//perform measurement an readinfg sensor-data
//duration about 70ms
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev) {
	//inividaul setting, refer BME280-doc
	dev->settings.osr_h = BME280_OVERSAMPLING_2X;
	dev->settings.osr_p = BME280_OVERSAMPLING_8X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_OFF;

	uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
	//set oversampling und filter
	int8_t rslt = bme280_set_sensor_settings(settings_sel, dev);
	uint32_t req_delay_ms = bme280_cal_meas_delay(&dev->settings);
	bme280_acquisition_time_ms = req_delay_ms;
	//initiate measurement in force-mode (inividual)
	rslt |= bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
	//wait for complete plus safty
	dev->delay_us((req_delay_ms + 5) * 1000, dev->intf_ptr);

	rslt |= bme280_get_sensor_data(BME280_ALL, &bme280_comp_data, dev);
	rslt |= bme280_set_sensor_mode(BME280_SLEEP_MODE, dev);

	return (rslt << 2);
}


int main (void) {
	bme280_cycles++;
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
