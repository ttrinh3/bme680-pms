/**
 * Wrapper module for source code compatibility with esp-open-rtos.
 */
#define ESP_PLATFORM
#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

#include <sys/time.h>
#include <string.h>



#include "driver/i2c.h"
#include "driver/gpio.h"

#include "esp8266_wrapper.h"

// esp-open-rtos SDK function wrapper

uint32_t sdk_system_get_time ()
{
    struct timeval time;
    gettimeofday(&time,0);
    return time.tv_sec*1e6 + time.tv_usec;
}

bool gpio_isr_service_installed = false;
bool auto_pull_up = false;
bool auto_pull_down = true;

esp_err_t gpio_set_interrupt(gpio_num_t gpio, 
                             gpio_int_type_t type, 
                             gpio_interrupt_handler_t handler)
{
    if (!gpio_isr_service_installed)
        gpio_isr_service_installed = (gpio_install_isr_service(0) == ESP_OK);

    gpio_config_t gpio_cfg = {
       .pin_bit_mask = ((uint64_t)(((uint64_t)1)<< gpio)),
       .mode = GPIO_MODE_INPUT,
       .pull_up_en = auto_pull_up,
       .pull_down_en = auto_pull_down,
       .intr_type = type
    };
    gpio_config(&gpio_cfg);

    // set interrupt handler
    gpio_isr_handler_add(gpio, (gpio_isr_t)handler, (void*)gpio);
    
    return ESP_OK;
}

void gpio_enable (gpio_num_t gpio, const gpio_mode_t mode)
{
    gpio_config_t gpio_cfg = {
       .pin_bit_mask = ((uint64_t)(((uint64_t)1)<< gpio)),
       .mode = mode,
       .pull_up_en = auto_pull_up,
       .pull_down_en = auto_pull_down,
    };
    gpio_config(&gpio_cfg);
}

// esp-open-rtos I2C interface wrapper

#define I2C_ACK_VAL  0x0
#define I2C_NACK_VAL 0x1

void i2c_init (int bus, gpio_num_t scl, gpio_num_t sda, uint32_t freq)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags=0;
    i2c_param_config(bus, &conf);
    i2c_driver_install(bus, I2C_MODE_MASTER, 0, 0, 0);
}
//i hardcoded bus to be 0, cause that's obviously the only one we'll use
int i2c_slave_write (uint8_t addr, 
                     uint8_t *data, uint32_t len,void *intf_ptr )
{
    uint8_t bus = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, addr << 1 | I2C_MASTER_WRITE, true);

    if (data)
        i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return err;
}

uint8_t i2c_slave_read ( uint8_t addr,  
                    uint8_t *data, uint32_t len,void *intf_ptr)
{
    uint8_t bus=0;
    if (len == 0) return true;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (data)
    {
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ( addr << 1 ) | I2C_MASTER_READ, true);
        if (len > 1) i2c_master_read(cmd, data, len-1, I2C_ACK_VAL);
        i2c_master_read_byte(cmd, data + len-1, I2C_NACK_VAL);
        i2c_master_stop(cmd);
    }
    esp_err_t err = i2c_master_cmd_begin(bus, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

// esp-open-rtos SPI interface wrapper





#endif  // ESP32 (ESP-IDF)

