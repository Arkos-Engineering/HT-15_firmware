#if !defined(RFMODULE_2M70CM_H)
#define RFMODULE_2M70CM_H
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct  {
    u8 spi_port;             /**< SPI port: 0 or 1 */
    u8 spi_pin_mosi;             /**< MOSI pin */
    u8 spi_pin_miso;             /**< MISO pin */
    u8 spi_pin_sck;              /**< SCK pin */
    u8 spi_pin_cs;               /**< CS pin */
    u32 spi_baudrate;        /**< SPI clock frequency in Hz */
    bool8 spi_shared;        /**< Whether the SPI bus is shared with other devices (e.g. display). if true, the SPI bus is expected to be initalized before use and will not be initialized by the RF module */


    u8 i2c_port;         /**< I2C port: 0 or 1 */
    u8 i2c_pin_sda;             /**< I2C SDA pin */
    u8 i2c_pin_scl;             /**< I2C SCL pin */
    u32 i2c_baudrate;        /**< I2C clock frequency in Hz */
    bool8 i2s_shared;       /**< Whether the I2S pins are shared with other devices (e.g. audio amp) if true, the SPI bus is expected to be initalized before use and will not be initialized by the RF module */
    
    u8 pin_gpio0;           
    u8 pin_gpio1;
    u8 pin_gpio2;
    u8 pin_gpio3;
} rfmodule_config_t; ;


typedef enum {
    RFMODULE_2M70CM_POWER_MODE_OFF = 0,
    RFMODULE_2M70CM_POWER_MODE_ON = 1,
    RFMODULE_2M70CM_POWER_MODE_RX_ONLY = 2,
} rfmodule_power_mode_t;


i8 rfmodule_2m70cm_init(rfmodule_config_t *dev);
i8 rfmodule_2m70cm_set_power_mode(rfmodule_config_t *dev, rfmodule_power_mode_t mode);

#if defined(RFMODULE_2M70CM_IMPLEMENTATION)
i8 rfmodule_2m70cm_init(rfmodule_config_t *dev){
    gpio_init(dev->pin_gpio0);
    gpio_set_dir(dev->pin_gpio0, GPIO_OUT);
    gpio_init(dev->pin_gpio1);
    gpio_set_dir(dev->pin_gpio1, GPIO_OUT);
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

#endif /* HT15_RFMODULE_2M70CM_IMPLEMENTATION */
#ifdef __cplusplus
}
#endif
#endif /* RFMODULE_2M70CM_H */