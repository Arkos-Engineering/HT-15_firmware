#if !defined(RFMODULE_2M70CM_H)
#define RFMODULE_2M70CM_H
#include "definitions.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STATE_IDLE = 0,
    STATE_RX = 1,
    STATE_TX = 2,
    STATE_FSTXON = 3,
    STATE_CALIBRATE = 4,
    STATE_SETTLING = 5,
    STATE_RXFIFO_OVERFLOW = 6,
    STATE_TXFIFO_UNDERFLOW = 7
} rfmodule_2m70cm_status_t;

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
} rfmodule_2m70cm_config_t;

typedef struct {
    rfmodule_2m70cm_config_t config;
    rfmodule_2m70cm_status_t state_status_byte; //cached copy of state byte read from CC1200, updated by sending commands or load_status_byte()
    bool8 chip_ready; //cached copy of ready bit from CC1200 Chip Status Byte. 0 if chip is ready, 1 if power and crystal are still stabalizing. update by sending commands or load_status_byte()
    bool8 is_keyed;
} rfmodule_2m70cm_state_t;

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
}rfmodule_2m70cm_cmd_strobe_t;

// CC1200 register addresses. This is AI generated. Verify against datasheet if any issues arise
typedef enum {
    /* normal register space */
    CC1200_REG_IOCFG3           = 0x0000,
    CC1200_REG_IOCFG2           = 0x0001,
    CC1200_REG_IOCFG1           = 0x0002,
    CC1200_REG_IOCFG0           = 0x0003,
    CC1200_REG_SYNC3            = 0x0004,
    CC1200_REG_SYNC2            = 0x0005,
    CC1200_REG_SYNC1            = 0x0006,
    CC1200_REG_SYNC0            = 0x0007,
    CC1200_REG_SYNC_CFG1        = 0x0008,
    CC1200_REG_SYNC_CFG0        = 0x0009,
    CC1200_REG_DEVIATION_M      = 0x000A,
    CC1200_REG_MODCFG_DEV_E     = 0x000B,
    CC1200_REG_DCFILT_CFG       = 0x000C,
    CC1200_REG_PREAMBLE_CFG1    = 0x000D,
    CC1200_REG_PREAMBLE_CFG0    = 0x000E,
    CC1200_REG_IQIC             = 0x000F,
    CC1200_REG_CHAN_BW          = 0x0010,
    CC1200_REG_MDMCFG1          = 0x0011,
    CC1200_REG_MDMCFG0          = 0x0012,
    CC1200_REG_SYMBOL_RATE2     = 0x0013,
    CC1200_REG_SYMBOL_RATE1     = 0x0014,
    CC1200_REG_SYMBOL_RATE0     = 0x0015,
    CC1200_REG_AGC_REF          = 0x0016,
    CC1200_REG_AGC_CS_THR       = 0x0017,
    CC1200_REG_AGC_GAIN_ADJUST  = 0x0018,
    CC1200_REG_AGC_CFG3         = 0x0019,
    CC1200_REG_AGC_CFG2         = 0x001A,
    CC1200_REG_AGC_CFG1         = 0x001B,
    CC1200_REG_AGC_CFG0         = 0x001C,
    CC1200_REG_FIFO_CFG         = 0x001D,
    CC1200_REG_DEV_ADDR         = 0x001E,
    CC1200_REG_SETTLING_CFG     = 0x001F,
    CC1200_REG_FS_CFG           = 0x0020,
    CC1200_REG_WOR_CFG1         = 0x0021,
    CC1200_REG_WOR_CFG0         = 0x0022,
    CC1200_REG_WOR_EVENT0_MSB   = 0x0023,
    CC1200_REG_WOR_EVENT0_LSB   = 0x0024,
    CC1200_REG_RXDCM_TIME       = 0x0025,
    CC1200_REG_PKT_CFG2         = 0x0026,
    CC1200_REG_PKT_CFG1         = 0x0027,
    CC1200_REG_PKT_CFG0         = 0x0028,
    CC1200_REG_RFEND_CFG1       = 0x0029,
    CC1200_REG_RFEND_CFG0       = 0x002A,
    CC1200_REG_PA_CFG1          = 0x002B,
    CC1200_REG_PA_CFG0          = 0x002C,
    CC1200_REG_ASK_CFG          = 0x002D,
    CC1200_REG_PKT_LEN          = 0x002E,

    /* extended register space */
    CC1200_REG_IF_MIX_CFG       = 0x2F00,
    CC1200_REG_FREQOFF_CFG      = 0x2F01,
    CC1200_REG_TOC_CFG          = 0x2F02,
    CC1200_REG_MARC_SPARE       = 0x2F03,
    CC1200_REG_ECG_CFG          = 0x2F04,
    CC1200_REG_MDMCFG2          = 0x2F05,
    CC1200_REG_EXT_CTRL         = 0x2F06,
    CC1200_REG_RCCAL_FINE       = 0x2F07,
    CC1200_REG_RCCAL_COARSE     = 0x2F08,
    CC1200_REG_RCCAL_OFFSET     = 0x2F09,
    CC1200_REG_FREQOFF1         = 0x2F0A,
    CC1200_REG_FREQOFF0         = 0x2F0B,
    CC1200_REG_FREQ2            = 0x2F0C,
    CC1200_REG_FREQ1            = 0x2F0D,
    CC1200_REG_FREQ0            = 0x2F0E,
    CC1200_REG_IF_ADC2          = 0x2F0F,
    CC1200_REG_IF_ADC1          = 0x2F10,
    CC1200_REG_IF_ADC0          = 0x2F11,
    CC1200_REG_FS_DIG1          = 0x2F12,
    CC1200_REG_FS_DIG0          = 0x2F13,
    CC1200_REG_FS_CAL3          = 0x2F14,
    CC1200_REG_FS_CAL2          = 0x2F15,
    CC1200_REG_FS_CAL1          = 0x2F16,
    CC1200_REG_FS_CAL0          = 0x2F17,
    CC1200_REG_FS_CHP           = 0x2F18,
    CC1200_REG_FS_DIVTWO        = 0x2F19,
    CC1200_REG_FS_DSM1          = 0x2F1A,
    CC1200_REG_FS_DSM0          = 0x2F1B,
    CC1200_REG_FS_DVC1          = 0x2F1C,
    CC1200_REG_FS_DVC0          = 0x2F1D,
    CC1200_REG_FS_LBI           = 0x2F1E,
    CC1200_REG_FS_PFD           = 0x2F1F,
    CC1200_REG_FS_PRE           = 0x2F20,
    CC1200_REG_FS_REG_DIV_CML   = 0x2F21,
    CC1200_REG_FS_SPARE         = 0x2F22,
    CC1200_REG_FS_VCO4          = 0x2F23,
    CC1200_REG_FS_VCO3          = 0x2F24,
    CC1200_REG_FS_VCO2          = 0x2F25,
    CC1200_REG_FS_VCO1          = 0x2F26,
    CC1200_REG_FS_VCO0          = 0x2F27,
    CC1200_REG_GBIAS6           = 0x2F28,
    CC1200_REG_GBIAS5           = 0x2F29,
    CC1200_REG_GBIAS4           = 0x2F2A,
    CC1200_REG_GBIAS3           = 0x2F2B,
    CC1200_REG_GBIAS2           = 0x2F2C,
    CC1200_REG_GBIAS1           = 0x2F2D,
    CC1200_REG_GBIAS0           = 0x2F2E,
    CC1200_REG_IFAMP            = 0x2F2F,
    CC1200_REG_LNA              = 0x2F30,
    CC1200_REG_RXMIX            = 0x2F31,
    CC1200_REG_XOSC5            = 0x2F32,
    CC1200_REG_XOSC4            = 0x2F33,
    CC1200_REG_XOSC3            = 0x2F34,
    CC1200_REG_XOSC2            = 0x2F35,
    CC1200_REG_XOSC1            = 0x2F36,
    CC1200_REG_XOSC0            = 0x2F37,
    CC1200_REG_ANALOG_SPARE     = 0x2F38,
    CC1200_REG_PA_CFG3          = 0x2F39,
    CC1200_REG_WOR_TIME1        = 0x2F64,
    CC1200_REG_WOR_TIME0        = 0x2F65,
    CC1200_REG_WOR_CAPTURE1     = 0x2F66,
    CC1200_REG_WOR_CAPTURE0     = 0x2F67,
    CC1200_REG_BIST             = 0x2F68,
    CC1200_REG_DCFILTOFFSET_I1  = 0x2F69,
    CC1200_REG_DCFILTOFFSET_I0  = 0x2F6A,
    CC1200_REG_DCFILTOFFSET_Q1  = 0x2F6B,
    CC1200_REG_DCFILTOFFSET_Q0  = 0x2F6C,
    CC1200_REG_IQIE_I1          = 0x2F6D,
    CC1200_REG_IQIE_I0          = 0x2F6E,
    CC1200_REG_IQIE_Q1          = 0x2F6F,
    CC1200_REG_IQIE_Q0          = 0x2F70,
    CC1200_REG_RSSI1            = 0x2F71,
    CC1200_REG_RSSI0            = 0x2F72,
    CC1200_REG_MARCSTATE        = 0x2F73,
    CC1200_REG_LQI_VAL          = 0x2F74,
    CC1200_REG_PQT_SYNC_ERR     = 0x2F75,
    CC1200_REG_DEM_STATUS       = 0x2F76,
    CC1200_REG_FREQOFF_EST1     = 0x2F77,
    CC1200_REG_FREQOFF_EST0     = 0x2F78,
    CC1200_REG_AGC_GAIN3        = 0x2F79,
    CC1200_REG_AGC_GAIN2        = 0x2F7A,
    CC1200_REG_AGC_GAIN1        = 0x2F7B,
    CC1200_REG_AGC_GAIN0        = 0x2F7C,
    CC1200_REG_CFM_RX_DATA_OUT  = 0x2F7D,
    CC1200_REG_CFM_TX_DATA_IN   = 0x2F7E,
    CC1200_REG_ASK_SOFT_RX_DATA = 0x2F7F,
    CC1200_REG_RNDGEN           = 0x2F80,
    CC1200_REG_MAGN2            = 0x2F81,
    CC1200_REG_MAGN1            = 0x2F82,
    CC1200_REG_MAGN0            = 0x2F83,
    CC1200_REG_ANG1             = 0x2F84,
    CC1200_REG_ANG0             = 0x2F85,
    CC1200_REG_CHFILT_I2        = 0x2F86,
    CC1200_REG_CHFILT_I1        = 0x2F87,
    CC1200_REG_CHFILT_I0        = 0x2F88,
    CC1200_REG_CHFILT_Q2        = 0x2F89,
    CC1200_REG_CHFILT_Q1        = 0x2F8A,
    CC1200_REG_CHFILT_Q0        = 0x2F8B,
    CC1200_REG_GPIO_STATUS      = 0x2F8C,
    CC1200_REG_FSCAL_CTRL       = 0x2F8D,
    CC1200_REG_PHASE_ADJUST     = 0x2F8E,
    CC1200_REG_PARTNUMBER       = 0x2F8F,
    CC1200_REG_PARTVERSION      = 0x2F90,
    CC1200_REG_SERIAL_STATUS    = 0x2F91,
    CC1200_REG_MODEM_STATUS1    = 0x2F92,
    CC1200_REG_MODEM_STATUS0    = 0x2F93,
    CC1200_REG_MARC_STATUS1     = 0x2F94,
    CC1200_REG_MARC_STATUS0     = 0x2F95,
    CC1200_REG_PA_IFAMP_TEST    = 0x2F96,
    CC1200_REG_FSRF_TEST        = 0x2F97,
    CC1200_REG_PRE_TEST         = 0x2F98,
    CC1200_REG_PRE_OVR          = 0x2F99,
    CC1200_REG_ADC_TEST         = 0x2F9A,
    CC1200_REG_DVC_TEST         = 0x2F9B,
    CC1200_REG_ATEST            = 0x2F9C,
    CC1200_REG_ATEST_LVDS       = 0x2F9D,
    CC1200_REG_ATEST_MODE       = 0x2F9E,
    CC1200_REG_XOSC_TEST1       = 0x2F9F,
    CC1200_REG_XOSC_TEST0       = 0x2FA0,
    CC1200_REG_AES              = 0x2FA1,
    CC1200_REG_MDM_TEST         = 0x2FA2,
    CC1200_REG_RXFIRST          = 0x2FD2,
    CC1200_REG_TXFIRST          = 0x2FD3,
    CC1200_REG_RXLAST           = 0x2FD4,
    CC1200_REG_TXLAST           = 0x2FD5,
    CC1200_REG_NUM_TXBYTES      = 0x2FD6,
    CC1200_REG_NUM_RXBYTES      = 0x2FD7,
    CC1200_REG_FIFO_NUM_TXBYTES = 0x2FD8,
    CC1200_REG_FIFO_NUM_RXBYTES = 0x2FD9,
    CC1200_REG_RXFIFO_PRE_BUF   = 0x2FDA,
    CC1200_REG_AES_KEY0         = 0x2FE0,
    CC1200_REG_AES_KEY1         = 0x2FE1,
    CC1200_REG_AES_KEY2         = 0x2FE2,
    CC1200_REG_AES_KEY3         = 0x2FE3,
    CC1200_REG_AES_KEY4         = 0x2FE4,
    CC1200_REG_AES_KEY5         = 0x2FE5,
    CC1200_REG_AES_KEY6         = 0x2FE6,
    CC1200_REG_AES_KEY7         = 0x2FE7,
    CC1200_REG_AES_KEY8         = 0x2FE8,
    CC1200_REG_AES_KEY9         = 0x2FE9,
    CC1200_REG_AES_KEY10        = 0x2FEA,
    CC1200_REG_AES_KEY11        = 0x2FEB,
    CC1200_REG_AES_KEY12        = 0x2FEC,
    CC1200_REG_AES_KEY13        = 0x2FED,
    CC1200_REG_AES_KEY14        = 0x2FEE,
    CC1200_REG_AES_KEY15        = 0x2FEF,
    CC1200_REG_AES_BUFFER0      = 0x2FF0,
    CC1200_REG_AES_BUFFER1      = 0x2FF1,
    CC1200_REG_AES_BUFFER2      = 0x2FF2,
    CC1200_REG_AES_BUFFER3      = 0x2FF3,
    CC1200_REG_AES_BUFFER4      = 0x2FF4,
    CC1200_REG_AES_BUFFER5      = 0x2FF5,
    CC1200_REG_AES_BUFFER6      = 0x2FF6,
    CC1200_REG_AES_BUFFER7      = 0x2FF7,
    CC1200_REG_AES_BUFFER8      = 0x2FF8,
    CC1200_REG_AES_BUFFER9      = 0x2FF9,
    CC1200_REG_AES_BUFFER10     = 0x2FFA,
    CC1200_REG_AES_BUFFER11     = 0x2FFB,
    CC1200_REG_AES_BUFFER12     = 0x2FFC,
    CC1200_REG_AES_BUFFER13     = 0x2FFD,
    CC1200_REG_AES_BUFFER14     = 0x2FFE,
    CC1200_REG_AES_BUFFER15     = 0x2FFF
} cc1200_reg_t;


u8 rfmodule_2m70cm_write_cmd(rfmodule_2m70cm_state_t *dev, rfmodule_2m70cm_cmd_strobe_t addr);
u8 rfmodule_2m70cm_write_register(rfmodule_2m70cm_state_t *dev, u16 addr, u8 value);
u8 rfmodule_2m70cm_read_register(rfmodule_2m70cm_state_t *dev, u16 addr);
i8 rfmodule_2m70cm_init(rfmodule_2m70cm_state_t *dev);
i8 rfmodule_2m70cm_hw_reset(rfmodule_2m70cm_state_t *dev);
i8 rfmodule_2m70cm_set_power_mode(rfmodule_2m70cm_state_t *dev, rfmodule_power_mode_t mode);
void rfmodule_2m70cm_set_frequency(rfmodule_2m70cm_state_t *dev, u32 frequency_hz);




#if defined(RFMODULE_2M70CM_IMPLEMENTATION)

void _rfmodule_2m70cm_process_status_byte(rfmodule_2m70cm_state_t *dev, u8 status){
    dev->chip_ready = ((status & (1 << 7)) == 0);
    dev->state_status_byte = ((status >> 4) & 0x7);
}

// SPI chip select 
void _rfmodule_2m70cm_set_cs(rfmodule_2m70cm_state_t *dev, bool8 value){
    gpio_put(dev->config.spi_pin_cs, value); /* active low CS */
}

i8 rfmodule_2m70cm_init(rfmodule_2m70cm_state_t *dev){

    //init CC1200 reset pin
    gpio_init(dev->config.pin_gpio0);
    gpio_set_dir(dev->config.pin_gpio0, GPIO_OUT);

    //init rf amp power control pin
    gpio_init(dev->config.pin_gpio1);
    gpio_set_dir(dev->config.pin_gpio1, GPIO_OUT);

    //init SPI if not shared, otherwise assume it is already initialized and configured correctly
    if(!dev->config.spi_shared){
        spi_init(dev->config.spi_port, dev->config.spi_baudrate);
        gpio_set_function(dev->config.spi_pin_mosi, GPIO_FUNC_SPI);
        gpio_set_function(dev->config.spi_pin_miso, GPIO_FUNC_SPI);
        gpio_set_function(dev->config.spi_pin_sck, GPIO_FUNC_SPI);
        gpio_init(dev->config.spi_pin_cs);
        gpio_set_dir(dev->config.spi_pin_cs, GPIO_OUT);
        gpio_put(dev->config.spi_pin_cs, 1); /* deselect */
    }
    
    //init I2C if not shared, otherwise assume it is already initialized and configured correctly
    if(!dev->config.i2c_shared){
        i2c_init(dev->config.i2c_port, dev->config.i2c_baudrate);
        gpio_set_function(dev->config.i2c_pin_sda, GPIO_FUNC_I2C);
        gpio_set_function(dev->config.i2c_pin_scl, GPIO_FUNC_I2C);
        gpio_pull_up(dev->config.i2c_pin_sda);
        gpio_pull_up(dev->config.i2c_pin_scl);
    }

    //power down RF amp and reset the module to start in a known state
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_OFF);
    rfmodule_2m70cm_hw_reset(dev);


    return 0;
}


u8 rfmodule_2m70cm_write_cmd(rfmodule_2m70cm_state_t *dev, rfmodule_2m70cm_cmd_strobe_t addr){
    u8 unprocessed_status = 0;
    _rfmodule_2m70cm_set_cs(dev, 0);
    spi_write_read_blocking(dev->config.spi_port, &addr, &unprocessed_status, 1);
    _rfmodule_2m70cm_set_cs(dev, 1);
    _rfmodule_2m70cm_process_status_byte(dev, unprocessed_status);
    return unprocessed_status;
}

u8 rfmodule_2m70cm_write_register(rfmodule_2m70cm_state_t *dev, u16 addr, u8 value){
    u8 txd[3] = {addr>>8, addr&0xFF, value}; //raw bytes to write
    u8 unprocessed_status = 0;

    _rfmodule_2m70cm_set_cs(dev, 0);
    if(txd[0] == 0){ // standard register space
        txd[1] = 0<<7 | 0<<6 | (txd[1]&0x3F); /* clear read bit (7), clear burst bit (6) on first byte sent (address byte in this case)*/
        spi_write_read_blocking(dev->config.spi_port, txd+1, &unprocessed_status, 1); //write txd[1] (address). It's a read because this gives us the status byte to process later
        spi_write_blocking(dev->config.spi_port, txd+2, 1); //write txd[2] (value)
	}
    else{ //extended register space, needs extended address byte
        txd[0] = 0<<7 | 0<<6 | (txd[0]&0x3F); /* clear read bit (7), clear burst bit (6) on first byte sent (set extended address space byte in this case)*/
        spi_write_read_blocking(dev->config.spi_port, txd, &unprocessed_status, 1); //write txd[0] (trigger extended address space). it's a read because this gives us the status byte to process later
        spi_write_blocking(dev->config.spi_port, txd+1, 2); //write txd[1] (address), and txd[2] (value)
	}

    _rfmodule_2m70cm_process_status_byte(dev, unprocessed_status);

	_rfmodule_2m70cm_set_cs(dev, 1);
    return value;
}

u8 rfmodule_2m70cm_read_register(rfmodule_2m70cm_state_t *dev, u16 addr){
    uint8_t txd[2] = {addr>>8, addr&0xFF};
    u8 unprocessed_status = 0;
    u8 value = 0;

    _rfmodule_2m70cm_set_cs(dev, 0);
    if(txd[0] == 0){
        txd[1] = 1<<7 | 0<<6 | (txd[1]&0x3F); /* set read bit (7), clear burst bit (6) on first byte sent (address byte in this case)*/
        spi_write_read_blocking(dev->config.spi_port, txd+1, &unprocessed_status, 1); //write txd[1] (address). It's a read because this gives us the status byte to process later
	}
    else{
        txd[0] = 1<<7 | 0<<6 | (txd[0]&0x3F); /* set read bit (7), clear burst bit (6) on first byte sent (set extended address space byte in this case)*/
        spi_write_read_blocking(dev->config.spi_port, txd, &unprocessed_status, 1); //write txd[0] (trigger extended address space). it's a read because this gives us the status byte to process later
        spi_write_blocking(dev->config.spi_port, txd+1, 1); //write txd[1] (address)
	}

    //read status byte from CC1200 and update cached status in driver state struct. This is necessary because the CC1200 replies with the status byte on every SPI access.
    _rfmodule_2m70cm_process_status_byte(dev, unprocessed_status);

    //read the requested register value
    spi_read_blocking(dev->config.spi_port, 0, &value, 1);
	_rfmodule_2m70cm_set_cs(dev, 1);

    return value;
}
// C++ examples from CC1200.cpp for reference when implementing the above functions in case there are any nuances to the CC1200 SPI interface that need to be accounted for. Note that the CC1200 is the same RF IC used on the 2M70CM module, so the SPI interface should be the same.
// uint8_t CC1200::readRegister(CC1200::Register reg)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(CC1200_READ | static_cast<uint8_t>(reg)));
// 	uint8_t regValue = spi.write(0);
// 	spi.deselect();
// 	return regValue;
// }
// uint8_t CC1200::readRegister(CC1200::ExtRegister reg)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(CC1200_READ | CC1200_EXT_ADDR));
// 	spi.write(static_cast<uint8_t>(reg));
// 	uint8_t regValue = spi.write(0);
// 	spi.deselect();
// 	return regValue;
// }
// void CC1200::writeRegister(Register reg, uint8_t value)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(CC1200_WRITE | static_cast<uint8_t>(reg)));
// 	spi.write(value);
// 	spi.deselect();
// }
// void CC1200::writeRegister(CC1200::ExtRegister reg, uint8_t value)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(CC1200_WRITE | CC1200_EXT_ADDR));
// 	spi.write(static_cast<uint8_t>(reg));
// 	spi.write(value);
// 	spi.deselect();
// }
// void CC1200::writeRegisters(CC1200::Register startReg, uint8_t const *values, size_t numRegisters)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(CC1200_WRITE | CC1200_BURST | static_cast<uint8_t>(startReg)));
// 	for(size_t byteIndex = 0; byteIndex < numRegisters; ++byteIndex)
// 	{
// 		spi.write(values[byteIndex]);
// 	}
// 	spi.deselect();
// }
// void CC1200::writeRegisters(CC1200::ExtRegister startReg, uint8_t const *values, size_t numRegisters)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(CC1200_WRITE | CC1200_BURST | CC1200_EXT_ADDR));
// 	spi.write(static_cast<uint8_t>(startReg));
// 	for(size_t byteIndex = 0; byteIndex < numRegisters; ++byteIndex)
// 	{
// 		spi.write(values[byteIndex]);
// 	}
// 	spi.deselect();
// }
// void CC1200::loadStatusByte(uint8_t status)
// {
// 	chipReady = !(status >> 7);
// 	state = static_cast<State>((status >> 4) & 0x7);
// }
// void CC1200::sendCommand(CC1200::Command command)
// {
// 	spi.select();
// 	loadStatusByte(spi.write(static_cast<uint8_t>(command)));
// 	spi.deselect();
// }

i8 rfmodule_2m70cm_set_power_mode(rfmodule_2m70cm_state_t *dev, rfmodule_power_mode_t mode){
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

i8 rfmodule_2m70cm_hw_reset(rfmodule_2m70cm_state_t *dev){
    gpio_put(dev->config.pin_gpio0, 0); /* assert reset */
    sleep_ms(10);
    gpio_put(dev->config.pin_gpio0, 1); /* deassert reset */
    sleep_ms(10);
    return 0;
}

void rfmodule_2m70cm_set_frequency(rfmodule_2m70cm_state_t *dev, uint32_t freq){
    static const u32 cc1200_xosc_hz = 40 * MHZ;
    static const u32 cc1200_freq_word_scale = 1 << 16;

    u8 fs_cfg_bandselect = 0;
    u8 lo_divider = 0;

    if(freq >= 820 * MHZ && freq <= 960 * MHZ){
        fs_cfg_bandselect = 0x02;
        lo_divider = 4;
    } else if(freq >= 410 * MHZ && freq <= 480 * MHZ){
        fs_cfg_bandselect = 0x04;
        lo_divider = 8;
    } else if(freq >= 273300 * KHZ && freq <= 320 * MHZ){
        fs_cfg_bandselect = 0x06;
        lo_divider = 12;
    } else if(freq >= 205 * MHZ && freq <= 240 * MHZ){
        fs_cfg_bandselect = 0x08;
        lo_divider = 16;
    } else if(freq >= 164 * MHZ && freq <= 192 * MHZ){
        fs_cfg_bandselect = 0x0A;
        lo_divider = 20;
    } else if(freq >= 136700 * KHZ && freq <= 160 * MHZ){
        fs_cfg_bandselect = 0x0B;
        lo_divider = 24;
    } else{
        return;
    }

    rfmodule_2m70cm_write_cmd(dev, SIDLE);
    sleep_us(100);

    {
        u32 freq_word = (u32)((((u64)freq * lo_divider * cc1200_freq_word_scale) + (cc1200_xosc_hz / 2)) / cc1200_xosc_hz);
        u8 next_fs_cfg = 0x10 | fs_cfg_bandselect; /* enable lock indicator and select synth band */

        /* Common synth settings taken from the working CC1200 reference driver in docs_resources. */
        rfmodule_2m70cm_write_register(dev, CC1200_REG_IF_ADC1,        0xEE);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_IF_ADC0,        0x10);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DIG1,        0x04);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_CAL1,        0x40);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_CAL0,        0x0E);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DIVTWO,      0x03);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DSM0,        0x33);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DVC1,        0xF7);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_PFD,         0x00);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_PRE,         0x6E);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_REG_DIV_CML, 0x1C);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_SPARE,       0xAC);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_VCO0,        0xB5);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_XOSC5,          0x0E);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_XOSC1,          0x03);

        if(lo_divider == 4){
            rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DIG0, 0x55);
            rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DVC0, 0x17);
            rfmodule_2m70cm_write_register(dev, CC1200_REG_IFAMP,   0x09);
        } else if(lo_divider == 8){
            rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DIG0, 0xA3);
            rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DVC0, 0x0F);
            rfmodule_2m70cm_write_register(dev, CC1200_REG_IFAMP,   0x0D);
        } else {
            /* Reference code uses the 164-192 MHz values as a fallback for lower bands. */
            rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DIG0, 0x50);
            rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_DVC0, 0x0F);
            rfmodule_2m70cm_write_register(dev, CC1200_REG_IFAMP,   0x0D);
        }

        rfmodule_2m70cm_write_register(dev, CC1200_REG_FS_CFG, next_fs_cfg);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FREQ2, (freq_word >> 16) & 0xFF);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FREQ1, (freq_word >> 8) & 0xFF);
        rfmodule_2m70cm_write_register(dev, CC1200_REG_FREQ0, freq_word & 0xFF);
    }
}

#endif /* HT15_RFMODULE_2M70CM_IMPLEMENTATION */
#ifdef __cplusplus
}
#endif
#endif /* RFMODULE_2M70CM_H */
