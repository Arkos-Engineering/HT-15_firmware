#if !defined(RFMODULE_2M70CM_H)
#define RFMODULE_2M70CM_H
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct  {
    spi_inst_t *spi_port;             /**< SPI port: spi0 or spi1 */
    u8 spi_pin_mosi;             /**< MOSI pin */
    u8 spi_pin_miso;             /**< MISO pin */
    u8 spi_pin_sck;              /**< SCK pin */
    u8 spi_pin_cs;               /**< CS pin */
    u32 spi_baudrate;        /**< SPI clock frequency in Hz */
    bool8 spi_shared;        /**< Whether the SPI bus is shared with other devices (e.g. display). if true, the SPI bus is expected to be initalized before use and will not be initialized by the RF module */


    i2c_inst_t *i2c_port;         /**< I2C port: i2c0 or i2c1 */
    u8 i2c_pin_sda;             /**< I2C SDA pin */
    u8 i2c_pin_scl;             /**< I2C SCL pin */
    u32 i2c_baudrate;        /**< I2C clock frequency in Hz */
    bool8 i2c_shared;       /**< Whether the I2C pins are shared with other devices (e.g. audio amp) if true, the I2C bus is expected to be initalized before use and will not be initialized by the RF module */
    
    u8 pin_gpio0;           
    u8 pin_gpio1;
    u8 pin_gpio2;
    u8 pin_gpio3;
} rfmodule_config_t; ;

typedef enum {
    RFMODULE_2M70CM_ERROR_NONE = 0,
    RFMODULE_2M_70CM_MISSING_MODULE = -1,
    RFMODULE_2M70CM_ERROR_INVALID_PARAM = -2,
    RFMODULE_2M70CM_ERROR_INIT_FAILED = -3,
} rfmodule_error_code_t;

typedef enum {
    RFMODULE_2M70CM_POWER_MODE_OFF = 0,
    RFMODULE_2M70CM_POWER_MODE_ON = 1,
    RFMODULE_2M70CM_POWER_MODE_RX_ONLY = 2,
} rfmodule_power_mode_t;

u8 rfmodule_2m70cm_set_cs(rfmodule_config_t *dev, bool8 value);
u8 rfmodule_2m70cm_write_register(rfmodule_config_t *dev, u16 addr, u8 value);
u8 rfmodule_2m70cm_read_register(rfmodule_config_t *dev, u16 addr);
i8 rfmodule_2m70cm_init(rfmodule_config_t *dev);
i8 rfmodule_2m70cm_set_power_mode(rfmodule_config_t *dev, rfmodule_power_mode_t mode);




#if defined(RFMODULE_2M70CM_IMPLEMENTATION)
// SPI chip select 
u8 rfmodule_2m70cm_set_cs(rfmodule_config_t *dev, bool8 value){
    gpio_put(dev->spi_pin_cs, value); /* active low CS */
}

u8 rfmodule_2m70cm_write_register(rfmodule_config_t *dev, u16 addr, u8 value){
    uint8_t txd[3] = {addr>>8, addr&0xFF, value};

    rfmodule_2m70cm_cs(dev, 0);

    if(txd[0] == 0){
        spi_write_blocking(dev->spi_port, txd+1, 2);
	}
    else{
        spi_write_blocking(dev->spi_port, txd, 3);
	}

	rfmodule_2m70cm_cs(dev, 1);
    return value;
}

u8 rfmodule_2m70cm_read_register(rfmodule_config_t *dev, u16 addr){
    uint8_t txd[2] = {addr>>8, addr&0xFF};
    u8 value = 0;

    rfmodule_2m70cm_cs(dev, 0);

    if(txd[0] == 0){
        spi_write_blocking(dev->spi_port, txd+1, 1);
	}
    else{
        spi_write_blocking(dev->spi_port, txd, 2);
	}

    spi_read_blocking(dev->spi_port, 0, &value, 1);

	rfmodule_2m70cm_cs(dev, 1);
    return value;

}

i8 rfmodule_2m70cm_init(rfmodule_config_t *dev){

    //init CC1200 reset pin
    gpio_init(dev->pin_gpio0);
    gpio_set_dir(dev->pin_gpio0, GPIO_OUT);

    //init rf amp power control pin
    gpio_init(dev->pin_gpio1);
    gpio_set_dir(dev->pin_gpio1, GPIO_OUT);

    //init SPI if not shared, otherwise assume it is already initialized and configured correctly
    if(!dev->spi_shared){
        spi_init(dev->spi_port, dev->spi_baudrate);
        gpio_set_function(dev->spi_pin_mosi, GPIO_FUNC_SPI);
        gpio_set_function(dev->spi_pin_miso, GPIO_FUNC_SPI);
        gpio_set_function(dev->spi_pin_sck, GPIO_FUNC_SPI);
        gpio_init(dev->spi_pin_cs);
        gpio_set_dir(dev->spi_pin_cs, GPIO_OUT);
        gpio_put(dev->spi_pin_cs, 1); /* deselect */
    }
    
    //init I2C if not shared, otherwise assume it is already initialized and configured correctly
    if(!dev->i2c_shared){
        i2c_init(dev->i2c_port, dev->i2c_baudrate);
        gpio_set_function(dev->i2c_pin_sda, GPIO_FUNC_I2C);
        gpio_set_function(dev->i2c_pin_scl, GPIO_FUNC_I2C);
        gpio_pull_up(dev->i2c_pin_sda);
        gpio_pull_up(dev->i2c_pin_scl);
    }

    //power down RF amp and reset the module to start in a known state
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_OFF);
    hw_reset(dev);


    return 0;
}

i8 rfmodule_2m70cm_set_power_mode(rfmodule_config_t *dev, rfmodule_power_mode_t mode){
    switch(mode){
        case RFMODULE_2M70CM_POWER_MODE_OFF:
            gpio_put(pin_rf_gpio1, 1);
            break;
        case RFMODULE_2M70CM_POWER_MODE_ON:
            gpio_put(pin_rf_gpio1, 0);
            break;
        case RFMODULE_2M70CM_POWER_MODE_RX_ONLY:
            gpio_put(pin_rf_gpio1, 1);
            break;
        default:
            return -1; /* invalid mode */
    }
    return 0;
}

i8 hw_reset(rfmodule_config_t *dev){
    gpio_put(dev->pin_gpio0, 0); /* assert reset */
    sleep_ms(10);
    gpio_put(dev->pin_gpio0, 1); /* deassert reset */
    sleep_ms(10);
    return 0;
}

#endif /* HT15_RFMODULE_2M70CM_IMPLEMENTATION */
#ifdef __cplusplus
}
#endif
#endif /* RFMODULE_2M70CM_H */