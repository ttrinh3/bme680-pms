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
#include "C:/Users/Timot/esp/esp-idf/components/ulp/ulp_riscv/include/ulp_riscv/ulp_riscv.h"
#include "C:/Users/Timot/esp/esp-idf/components/ulp/ulp_riscv/include/ulp_riscv/ulp_riscv_utils.h"
#include "C:/Users/Timot/esp/esp-idf/components/ulp/ulp_riscv/include/ulp_riscv/ulp_riscv_gpio.h"

// #include "BME280_driver-master/bme280.h"
// #include "BME280_driver-master/bme280_defs.h"
#include "BME280_driver-master/bme68x.h"
#include "BME280_driver-master/bme68x_defs.h"
/*
 * #define BME280_32BIT_ENABLE	!!! set in bme280_defs.h !!!
 */


//internal varibales
static uint8_t dev_addr;


//cache wakeup check




uint32_t bme280_chip_id = 0;

uint32_t bme280_sda = 0;
uint32_t bme280_scl = 0;

//test variable to be sent to main\
// check read_field_data(. looks like they're all uint8_ts
//the buff is uint8_t so that's what these should be for raw data
uint8_t tempLSB = 0;
uint8_t tempMSB = 0;
uint8_t tempXSB = 0;
//just for calibration data
//types are correct, check get_calib_data
//use these for calib

// uint16_t tempLSB = 0;
// int16_t tempMSB = 0;
// int8_t tempXSB = 0;

//temperature should be
//this is returned by the calculate temperature
int16_t temperature = 0;
//this holds the raw adc temp, look at 1138 of .c
uint32_t temp_temperature=0;
uint8_t coeff_array[BME68X_LEN_COEFF_ALL];
//TEST
// int32_t tempLSB = 0;
// int32_t tempMSB = 0;
// int32_t tempXSB = 0;




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


int8_t bme280_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr){
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


void bme680_i2c_init(struct bme68x_dev *dev) {
	dev_addr = BME68X_I2C_ADDR_HIGH;	//SDO 10k Pulldown on Sensorboard => addr = 0x76
	//for the above, the 680 and 280 have the exact same address
	//instead of primary and secondary in the 280, 
	//the 680 has the low and high, but values are the same.
	//here he loads the dev with self written functions 
	dev->intf_ptr = &dev_addr; //OK makes sense
	dev->intf  = BME68X_I2C_INTF; //OK makes sense
	dev->read  = bme280_i2c_read; //this seems like functions that he create and points to
	dev->write = bme280_i2c_write;//again, this is a self created function (OK)
	dev->delay_us = user_delay_us;//self created (defined in this file)
	//safe side -> SW-Reset
	const uint8_t com_res = BME68X_SOFT_RESET_CMD; //OK makes sense
	bme280_i2c_write(BME68X_REG_SOFT_RESET, &com_res, 1, dev); //OK makes sense
	user_delay_us(5*1000, NULL); // > 2ms PowerOn-Reset

	// return (bme68x_init(dev) << 1); //OK makes sense
}


        // dev->calib.par_t1 =
        //     (uint16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T1_MSB], coeff_array[BME68X_IDX_T1_LSB]));
        // dev->calib.par_t2 =
        //     (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T2_MSB], coeff_array[BME68X_IDX_T2_LSB]));
        // dev->calib.par_t3 = (int8_t)(coeff_array[BME68X_IDX_T3]);
static int16_t calc_temperature1(uint32_t temp_adc, struct bme68x_dev *dev)
{
    int64_t var1;
    int64_t var2;
    int64_t var3;
	int32_t t_fine;
    int16_t calc_temp;

    /*lint -save -e701 -e702 -e704 */
    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)26115 << 1);
    var2 = (var1 * (int32_t)26661) >> 11;
    var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
    var3 = ((var3) * ((int32_t)3 << 4)) >> 14;
    t_fine = (int32_t)(var2 + var3);
    calc_temp = (int16_t)(((t_fine * 5) + 128) >> 8);

    /*lint -restore */
    return calc_temp;
}

int main (void) {
	

	init_gpio();
	// int8_t rslt = 0;
	uint8_t chip_id = 2;
	struct bme68x_dev bme;
    struct bme68x_conf conf;
    struct bme68x_data data;

	// bme68x_init(&bme);
	bme680_i2c_init(&bme);
	// bme68x_get_regs(0xd0 , &chip_id, 1, &bme);

    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_1X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_1X;
     bme68x_set_conf(&conf, &bme);
	bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
	ulp_riscv_delay_cycles(10000 * 17.5);

	///////this part gets the raw temperature//////////
	//check where what teh type should be?
	//it should be uint8_t because that's what the buff is
	bme68x_get_regs(UINT8_C(0x22) , &tempMSB, 1, &bme);
	// tempMSB = chip_id;//5
	bme68x_get_regs(UINT8_C(0x23) , &tempLSB, 1, &bme);
	// tempLSB = chip_id;//6
	bme68x_get_regs(UINT8_C(0x24) , &tempXSB, 1, &bme);
	// tempXSB = chip_id;//7
	temp_temperature = (uint32_t)(((uint32_t)tempMSB * 4096) | ((uint32_t)tempLSB * 16) | ((uint32_t)tempXSB / 16));
	/////////////////////////////////////////////////////////////////////////////////////==========================
	// tempLSB = 3;
	//this temperature should be // uint32_t adc_temp; probably make another variable for it
	temperature = calc_temperature1(temp_temperature, &bme);
	ulp_riscv_delay_cycles(1000000 * 17.5);
	// temperature = 0;
//the below will obtain the calibration parameters
// bme68x_get_regs(BME68X_REG_COEFF1, coeff_array, BME68X_LEN_COEFF1, &bme);
// bme68x_get_regs(BME68X_REG_COEFF2, &coeff_array[BME68X_LEN_COEFF1], BME68X_LEN_COEFF2, &bme);
// bme68x_get_regs(BME68X_REG_COEFF3,
//                                &coeff_array[BME68X_LEN_COEFF1 + BME68X_LEN_COEFF2],
//                                BME68X_LEN_COEFF3,
//                                &bme);
// tempLSB = (uint16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T1_MSB], coeff_array[BME68X_IDX_T1_LSB])); //t1
// tempMSB = (int16_t)(BME68X_CONCAT_BYTES(coeff_array[BME68X_IDX_T2_MSB], coeff_array[BME68X_IDX_T2_LSB]));//t2
// tempXSB = (int8_t)(coeff_array[BME68X_IDX_T3]);//t3








	ulp_riscv_wakeup_main_processor();
}



