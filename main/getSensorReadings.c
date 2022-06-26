#include "getSensorReadings.h"
static bme680_sensor_t* sensor = 0;
void getSensorReadings(void){
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
                char buffer[500];
                sprintf(buffer," BME680 Sensor: %.2f °C, %.2f %%, %.2f hPa, %.2f Ohm\n",
                       values.temperature, values.humidity,
                       values.pressure, values.gas_resistance);
                    printf("%s\n",buffer);
                        pms5003_make_measurement(&pms0, &reading);
                        pms5003_print_measurement(&reading);       
                       
                       }
        }
        // passive waiting until 1 second is over
        vTaskDelayUntil(&last_wakeup, 1000 / portTICK_PERIOD_MS);
    }
    }
    else
        printf("Could not initialize BME680 sensor\n");
}
