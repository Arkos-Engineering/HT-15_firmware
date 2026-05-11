#if !defined(RFMODULE_2M70CM_H)
#define RFMODULE_2M70CM_H
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
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
} rfmodule_config_t;

typedef enum {
    SRES = 0x30, /* Reset chip. */
    SFSTXON,/* Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). */
    SXOFF, /* Turn off crystal oscillator. */
    SCAL, /* Calibrate frequency synthesizer and turn it off. */
    SRX, /* Enable RX. Perform calibration first if coming from */
    STX, /* Enable TX. Perform calibration first if coming from */
    SIDLE, /* Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. */
    SAFC, /* Perform AFC adjustment of the frequency synthesizer */
    SWOR, /* Start automatic RX polling sequence (Wake-on-Radio) */
    SPWD, /* Enter power down mode when CSn goes high. */
    SFRX, /* Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states. */
    SFTX, /* Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states. */
    SWORRST, /* Reset real time clock. Only issue SWORRST in IDLE state. */
    SNOP /* No operation. May be used to get access to the chip status byte. */
}rfmodule_2m70cm_cmd_strobe;

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
u8 rfmodule_2m70cm_write_cmd(rfmodule_config_t *dev, rfmodule_2m70cm_cmd_strobe addr);
u8 rfmodule_2m70cm_write_register(rfmodule_config_t *dev, u16 addr, u8 value);
u8 rfmodule_2m70cm_read_register(rfmodule_config_t *dev, u16 addr);
i8 rfmodule_2m70cm_init(rfmodule_config_t *dev);
i8 rfmodule_2m70cm_hw_reset(rfmodule_config_t *dev);
i8 rfmodule_2m70cm_set_power_mode(rfmodule_config_t *dev, rfmodule_power_mode_t mode);
void rfmodule_2m70cm_set_frequency(rfmodule_config_t *dev, u32 frequency_hz);

#if defined(RFMODULE_2M70CM_IMPLEMENTATION)
// SPI chip select 
u8 rfmodule_2m70cm_set_cs(rfmodule_config_t *dev, bool8 value){
    gpio_put(dev->spi_pin_cs, value); /* active low CS */
}

u8 rfmodule_2m70cm_write_cmd(rfmodule_config_t *dev, rfmodule_2m70cm_cmd_strobe addr){
    rfmodule_2m70cm_set_cs(dev, 0);
    spi_write_blocking(dev->spi_port, &addr, 1);
    rfmodule_2m70cm_set_cs(dev, 1);
}

u8 rfmodule_2m70cm_write_register(rfmodule_config_t *dev, u16 addr, u8 value){
    uint8_t txd[3] = {addr>>8, addr&0xFF, value};

    rfmodule_2m70cm_set_cs(dev, 0);

    if(txd[0] == 0){
        spi_write_blocking(dev->spi_port, txd+1, 2);
	}
    else{
        spi_write_blocking(dev->spi_port, txd, 3);
	}

	rfmodule_2m70cm_set_cs(dev, 1);
    return value;
}

u8 rfmodule_2m70cm_read_register(rfmodule_config_t *dev, u16 addr){
    uint8_t txd[2] = {addr>>8, addr&0xFF};
    u8 value = 0;

    rfmodule_2m70cm_set_cs(dev, 0);
    if(txd[0] == 0){
        spi_write_blocking(dev->spi_port, txd+1, 1);
	}
    else{
        spi_write_blocking(dev->spi_port, txd, 2);
	}
    spi_read_blocking(dev->spi_port, 0, &value, 1);
	rfmodule_2m70cm_set_cs(dev, 1);

    return value;

}
// C++ examples from CC1200.cpp for reference when implementing the above functions in case there are any nuances to the CC1200 SPI interface that need to be accounted for. Note that the CC1200 is the same RF IC used on the 2M70CM module, so the SPI interface should be the same.
// uint8_t CC1200::readRegister(CC1200::Register reg)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(1<<7 | static_cast<uint8_t>(reg)));
// 	uint8_t regValue = spi.write(0);
// 	spi.deselect();
// 	return regValue;
// }
// uint8_t CC1200::readRegister(CC1200::ExtRegister reg)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(1<<7 | 0x2F));
// 	spi.write(static_cast<uint8_t>(reg));
// 	uint8_t regValue = spi.write(0);
// 	spi.deselect();
// 	return regValue;
// }
// void CC1200::loadStatusByte(uint8_t status)
// {
// 	chipReady = !(status >> 7);
// 	state = static_cast<State>((status >> 4) & 0x7);
// }

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
    rfmodule_2m70cm_hw_reset(dev);


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

i8 rfmodule_2m70cm_hw_reset(rfmodule_config_t *dev){
    gpio_put(dev->pin_gpio0, 0); /* assert reset */
    sleep_ms(10);
    gpio_put(dev->pin_gpio0, 1); /* deassert reset */
    sleep_ms(10);
    return 0;
}

void rfmodule_2m70cm_set_frequency(rfmodule_config_t *dev, uint32_t freq){
	rfmodule_2m70cm_write_cmd(dev, SIDLE);
	sleep_ms(10);

	uint32_t freq_word = roundf((float)freq/5000000.0f*((uint32_t)1<<16));
	rfmodule_2m70cm_write_register(dev, 0x2F0C, (freq_word>>16)&0xFF);
	rfmodule_2m70cm_write_register(dev, 0x2F0D, (freq_word>>8)&0xFF);
	rfmodule_2m70cm_write_register(dev, 0x2F0E, freq_word&0xFF);
}

#endif /* HT15_RFMODULE_2M70CM_IMPLEMENTATION */
#ifdef __cplusplus
}
#endif
#endif /* RFMODULE_2M70CM_H */