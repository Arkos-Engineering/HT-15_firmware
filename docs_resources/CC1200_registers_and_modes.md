# CC1200 Register and Mode Guide for the HT-15 RF Module

Note: This document was written by AI. Use it with a grain of salt, and verify
critical register settings against the TI documentation and real hardware.

This document is a practical companion for the Arkos Engineering HT-15 RF module
code. It explains what each CC1200 register is for, how the important register
groups fit together, and how to bring the radio up for CW, custom FM, 2-FSK/FM,
and 4-FSK TX/RX.

Sources used:

- TI, CC120x Low-Power High-Performance Sub-1-GHz RF Transceivers User's Guide,
  SWRU346B: https://www.ti.com/lit/pdf/swru346
- TI, CC1200 product page and datasheet: https://www.ti.com/product/CC1200
- TI E2E note on CW mode: https://e2e.ti.com/support/wireless-connectivity/sub-1-ghz-group/sub-1-ghz/f/sub-1-ghz-forum/345737/how-to-set-cc1200-for-cw-transmission

## Scope and Safety

The CC1200 is a packet/data transceiver. It supports 2-FSK, 2-GFSK, 4-FSK,
4-GFSK, MSK, ASK, and OOK as normal modem modes. It also has Custom Frequency
Modulation (CFM), which lets firmware feed signed frequency-offset samples for
software-defined modulation. In this document:

- "CW" means an unmodulated RF carrier keyed on/off by firmware.
- "FM" can mean either normal 2-FSK/GFSK packet FM or custom analog-like FM
  using CFM samples. Both are covered.
- "4FSK" means the CC1200 packet modem 4-level FSK mode.

Only transmit on frequencies and power levels you are legally allowed to use.
For bring-up, use a dummy load or shielded test setup. The 146 MHz examples are
inside the US amateur 2 m band, but the CC1200/front-end/filter/PA chain still
must match the intended band and power level.

## HT-15 Code Conventions

The current HT-15 helper functions are:

```c
u8 rfmodule_2m70cm_write_cmd(rfmodule_config_t *dev, rfmodule_2m70cm_cmd_strobe addr);
u8 rfmodule_2m70cm_write_register(rfmodule_config_t *dev, u16 addr, u8 value);
u8 rfmodule_2m70cm_read_register(rfmodule_config_t *dev, u16 addr);
i8 rfmodule_2m70cm_set_power_mode(rfmodule_config_t *dev, rfmodule_power_mode_t mode);
```

Normal CC1200 registers are addressed as `0x00` through `0x2E`. Extended
registers are addressed by sending `0x2F` followed by the extended byte. The
HT-15 helper represents those as `0x2Fxx`, for example:

```c
rfmodule_2m70cm_write_register(&rfmodule_config, 0x2F0C, 0x57); /* FREQ2 */
rfmodule_2m70cm_write_register(&rfmodule_config, 0x2F05, 0x01); /* MDMCFG2 */
```

Packet TX/RX examples below use FIFO burst access. The current helper header
does not yet expose FIFO burst helpers, so the examples include placeholder
helpers named `cc1200_write_fifo()` and `cc1200_read_fifo()`. Implement those
with SPI burst access to address `0x3F`.

The CC1200 SPI header uses bit 7 for read and bit 6 for burst. If register
reads or FIFO access behave strangely, confirm the low-level helper is setting
those bits for read/burst transactions and is handling extended reads through
the `0x2F` address prefix.

FIFO helper sketch:

```c
#define CC1200_SPI_READ  0x80
#define CC1200_SPI_BURST 0x40
#define CC1200_FIFO_ADDR 0x3F

static void cc1200_write_fifo(rfmodule_config_t *dev, const u8 *data, u8 len)
{
    u8 header = CC1200_FIFO_ADDR | CC1200_SPI_BURST;
    rfmodule_2m70cm_set_cs(dev, 0);
    spi_write_blocking(dev->spi_port, &header, 1);
    spi_write_blocking(dev->spi_port, data, len);
    rfmodule_2m70cm_set_cs(dev, 1);
}

static void cc1200_read_fifo(rfmodule_config_t *dev, u8 *data, u8 len)
{
    u8 header = CC1200_FIFO_ADDR | CC1200_SPI_READ | CC1200_SPI_BURST;
    rfmodule_2m70cm_set_cs(dev, 0);
    spi_write_blocking(dev->spi_port, &header, 1);
    spi_read_blocking(dev->spi_port, 0, data, len);
    rfmodule_2m70cm_set_cs(dev, 1);
}
```

## Address Spaces and Commands

The CC1200 SPI map has four main regions:

| Address | Region | Purpose |
| --- | --- | --- |
| `0x00` to `0x2E` | Normal registers | Primary configuration registers. These retain values through sleep modes that retain configuration. |
| `0x2Fxx` | Extended registers | Additional configuration, calibration, status, test, AES, FIFO pointer, and soft-sample registers. |
| `0x30` to `0x3D` | Command strobes | One-byte commands such as reset, calibrate, RX, TX, idle, flush FIFO. |
| `0x3F` | FIFO | TX FIFO when written, RX FIFO when read. Burst access is normally used. |

Command strobes:

| Strobe | Hex | What it does |
| --- | ---: | --- |
| `SRES` | `0x30` | Reset CC1200 digital core. |
| `SFSTXON` | `0x31` | Start and settle the synthesizer without entering TX. Useful for fast TX later. |
| `SXOFF` | `0x32` | Turn off the crystal oscillator. |
| `SCAL` | `0x33` | Calibrate the frequency synthesizer. Normally issue from IDLE. |
| `SRX` | `0x34` | Enter receive mode. |
| `STX` | `0x35` | Enter transmit mode. |
| `SIDLE` | `0x36` | Leave RX/TX and return to IDLE. |
| `SAFC` | `0x37` | Apply automatic frequency correction on next active transition. |
| `SWOR` | `0x38` | Start Wake-On-Radio polling. |
| `SPWD` | `0x39` | Enter power down when CSn goes high. |
| `SFRX` | `0x3A` | Flush RX FIFO. Use only in IDLE or RX overflow states. |
| `SFTX` | `0x3B` | Flush TX FIFO. Use only in IDLE or TX underflow states. |
| `SWORRST` | `0x3C` | Reset the eWOR timer. |
| `SNOP` | `0x3D` | No operation. Commonly used to read the chip status byte. |

## Core Configuration Formulas

These are the formulas you will use most often when creating register settings
by firmware.

### RF Frequency

The carrier frequency is set by:

```text
FREQ = round(f_rf_hz * lo_divider * 2^16 / f_xosc_hz)
```

Then write:

```text
FREQ2 = FREQ[23:16]
FREQ1 = FREQ[15:8]
FREQ0 = FREQ[7:0]
```

`FS_CFG.FSD_BANDSELECT` selects the LO divider:

| RF band | LO divider | `FS_CFG` band bits |
| --- | ---: | ---: |
| 820 to 960 MHz | 4 | `0x02` |
| 410 to 480 MHz | 8 | `0x04` |
| 273.3 to 320 MHz | 12 | `0x06` |
| 205 to 240 MHz | 16 | `0x08` |
| 164 to 192 MHz | 20 | `0x0A` |
| 136.7 to 160 MHz | 24 | `0x0B` |

For 146.000 MHz with a 40 MHz crystal:

```text
lo_divider = 24
FREQ = round(146000000 * 24 * 65536 / 40000000) = 0x57999A
FREQ2 = 0x57
FREQ1 = 0x99
FREQ0 = 0x9A
FS_CFG = 0x0B
```

### Frequency Deviation

Deviation is controlled by `DEVIATION_M.DEV_M` and
`MODCFG_DEV_E.DEV_E`.

For `DEV_E > 0`:

```text
f_dev = f_xosc_hz * (256 + DEV_M) * 2^DEV_E / 2^22
```

For `DEV_E == 0`:

```text
f_dev = f_xosc_hz * DEV_M / 2^21
```

Useful 40 MHz examples:

| Target deviation | `DEV_E` | `DEV_M` | Actual |
| ---: | ---: | ---: | ---: |
| 2.4 kHz | `0` | `126` | 2403.26 Hz |
| 5.0 kHz | `1` | `6` | 4997.25 Hz |
| 9.6 kHz | `1` | `247` | 9593.96 Hz |
| 12.5 kHz | `2` | `72` | 12512.21 Hz |
| 25 kHz | `3` | `72` | 25024.41 Hz |

### Symbol Rate

The symbol rate is controlled by `SYMBOL_RATE2.SRATE_E` and the 20-bit
`SRATE_M` spread across `SYMBOL_RATE2[3:0]`, `SYMBOL_RATE1`, and
`SYMBOL_RATE0`.

For `SRATE_E > 0`:

```text
R_symbol = f_xosc_hz * (2^20 + SRATE_M) * 2^SRATE_E / 2^39
```

Then:

```text
SYMBOL_RATE2 = (SRATE_E << 4) | ((SRATE_M >> 16) & 0x0F)
SYMBOL_RATE1 = (SRATE_M >> 8) & 0xFF
SYMBOL_RATE0 = SRATE_M & 0xFF
```

With a 40 MHz crystal:

| Target symbol rate | `SRATE_E` | `SRATE_M` | Register bytes |
| ---: | ---: | ---: | --- |
| 4800 sym/s | `5` | `0xF7510` | `5F 75 10` |
| 9600 sym/s | `6` | `0xF7510` | `6F 75 10` |
| 12500 sym/s | `7` | `0x47AE1` | `74 7A E1` |
| 25000 sym/s | `8` | `0x47AE1` | `84 7A E1` |

For 2-FSK, bit rate equals symbol rate. For 4-FSK, each symbol carries two
bits, so a 9600 bit/s link uses 4800 symbols/s.

## Register Map

The tables below cover each named register in the CC1200 map. Many analog,
calibration, and test registers contain TI-reserved fields. For those registers,
the practical rule is: start from SmartRF Studio generated values for your band,
rate, deviation, RX bandwidth, and packet format, then change only the fields
your firmware owns.

### Normal Register Space

| Address | Register | What it controls | Practical notes |
| ---: | --- | --- | --- |
| `0x00` | `IOCFG3` | GPIO3 function, drive/inversion, and output source. | Use for GDO/status routing, external 40 kHz clock, or high-Z. Board wiring decides whether this is useful. |
| `0x01` | `IOCFG2` | GPIO2 function, drive/inversion, and output source. | Commonly route sync detect, packet done, FIFO threshold, CCA, or clock output. |
| `0x02` | `IOCFG1` | GPIO1 function, drive/inversion, and output source. | On HT-15 this physical pin may also be used by module power/PA control, so check hardware before assigning CC1200 status here. |
| `0x03` | `IOCFG0` | GPIO0 function, drive/inversion, and output source. | On HT-15 this is used as the CC1200 reset line by the module code, so do not also depend on it as a CC1200 GDO output unless hardware confirms it is connected that way. |
| `0x04` | `SYNC3` | Sync word byte 3, most significant byte. | Used by packet RX to identify frame start. Ignored for pure CW/RSSI reception. |
| `0x05` | `SYNC2` | Sync word byte 2. | Keep TX and RX sides identical. |
| `0x06` | `SYNC1` | Sync word byte 1. | Sync length and strictness are controlled by `SYNC_CFGx`. |
| `0x07` | `SYNC0` | Sync word byte 0, least significant byte. | Use nontrivial sync values to avoid false locks. |
| `0x08` | `SYNC_CFG1` | Sync-detect qualifier behavior. | Controls sync threshold/qualifier style and related preamble/sync handling. Use SmartRF Studio defaults unless changing packet acquisition. |
| `0x09` | `SYNC_CFG0` | Sync format and detection mode. | Also affects special RX modes and ASK/OOK configuration limits. This is critical for packet RX. |
| `0x0A` | `DEVIATION_M` | Frequency deviation mantissa. | Pairs with `MODCFG_DEV_E.DEV_E`. For CFM, this sets the scale for `CFM_TX_DATA_IN` and `CFM_RX_DATA_OUT`. |
| `0x0B` | `MODCFG_DEV_E` | Modulation format and deviation exponent. | Bits select 2-FSK, 2-GFSK, ASK/OOK, 4-FSK, or 4-GFSK. Low bits are `DEV_E`. |
| `0x0C` | `DCFILT_CFG` | Digital DC offset removal. | Important for RX demod stability. Freeze/enable options affect settling and low-frequency content. |
| `0x0D` | `PREAMBLE_CFG1` | Preamble length and preamble quality behavior. | Longer preambles help AGC, timing, and frequency correction settle. |
| `0x0E` | `PREAMBLE_CFG0` | Preamble detector threshold and preamble pattern controls. | Use stricter detection for noisy channels; relax for weak or short preambles. |
| `0x0F` | `IQIC` | IQ image compensation. | Helps suppress image response in RX. Typically left at SmartRF Studio value. |
| `0x10` | `CHAN_BW` | RX channel filter bandwidth and decimation. | Must cover data modulation, frequency error, and drift. Too narrow loses packets; too wide admits noise. |
| `0x11` | `MDMCFG1` | Main modem behavior such as Manchester/FEC and carrier-sense gating. | Affects packet coding and acquisition. TX/RX peers must match coding choices. |
| `0x12` | `MDMCFG0` | Additional modem behavior, transparent/serial-related options, and reserved modem fields. | Treat as a generated modem register. Change only when intentionally using transparent/serial/CFM features. |
| `0x13` | `SYMBOL_RATE2` | Symbol-rate exponent and upper mantissa nibble. | Program with `SYMBOL_RATE1/0`. For 4-FSK, symbol rate is half bit rate. |
| `0x14` | `SYMBOL_RATE1` | Symbol-rate mantissa middle byte. | See formula above. |
| `0x15` | `SYMBOL_RATE0` | Symbol-rate mantissa low byte. | See formula above. |
| `0x16` | `AGC_REF` | AGC reference level. | Sets the target level the receiver AGC tries to maintain. |
| `0x17` | `AGC_CS_THR` | Carrier-sense threshold. | Determines when the channel is considered occupied. Useful for RSSI/CW detect and LBT. |
| `0x18` | `AGC_GAIN_ADJUST` | Manual AGC gain offset/adjustment. | Fine-tunes front-end gain behavior. Normally generated. |
| `0x19` | `AGC_CFG3` | AGC dynamic behavior. | Controls settling, hysteresis, and gain loop behavior. Generated values matter a lot for sensitivity. |
| `0x1A` | `AGC_CFG2` | AGC behavior and gain limits. | Generated. Change carefully when optimizing blocking or weak-signal RX. |
| `0x1B` | `AGC_CFG1` | AGC behavior during acquisition/tracking. | Generated. Often tied to RX bandwidth and data rate. |
| `0x1C` | `AGC_CFG0` | AGC filter/loop details. | Generated. Bad values can make the receiver deaf or unstable. |
| `0x1D` | `FIFO_CFG` | TX/RX FIFO threshold configuration. | Thresholds can drive GPIO interrupts. Useful for streaming packets larger than FIFO depth. |
| `0x1E` | `DEV_ADDR` | Device address used by address filtering. | Packet mode only. Ignored if address filtering is disabled. |
| `0x1F` | `SETTLING_CFG` | Synthesizer autocalibration and settling timing. | Controls whether calibration happens on IDLE-to-RX/TX and lock timing. |
| `0x20` | `FS_CFG` | Frequency synthesizer band select and lock detector. | Set band bits before programming frequency. 146 MHz uses band value `0x0B`. |
| `0x21` | `WOR_CFG1` | Wake-On-Radio timing resolution and behavior. | Low-power polling only. Not needed for normal continuous RX. |
| `0x22` | `WOR_CFG0` | Wake-On-Radio event and RC oscillator behavior. | Low-power polling only. |
| `0x23` | `WOR_EVENT0_MSB` | eWOR event timeout high byte. | Sets polling interval with `WOR_EVENT0_LSB`. |
| `0x24` | `WOR_EVENT0_LSB` | eWOR event timeout low byte. | Sets polling interval with `WOR_EVENT0_MSB`. |
| `0x25` | `RXDCM_TIME` | RX duty-cycle mode time. | Determines how long RX stays on in RX duty-cycle mode. |
| `0x26` | `PKT_CFG2` | Packet engine options. | Controls parts of packet qualification, CCA, and packet handling. Generated values should match packet format. |
| `0x27` | `PKT_CFG1` | Packet format features. | Enables CRC, appended status, address check, and similar packet-engine behavior depending on field values. |
| `0x28` | `PKT_CFG0` | Packet length/configuration mode. | Selects fixed/variable/infinite/transparent packet length style. |
| `0x29` | `RFEND_CFG1` | RX end behavior. | Selects what state the radio enters after RX events such as packet done, timeout, or error. |
| `0x2A` | `RFEND_CFG0` | TX end behavior and CCA/TX transition behavior. | Selects what happens after TX and how TX interacts with clear-channel assessment. |
| `0x2B` | `PA_CFG1` | PA ramp enable and output power ramp target. | Main output power register. Board PA and regulatory limits matter. |
| `0x2C` | `PA_CFG0` | PA intermediate ramp levels and ramp shape. | Controls spectral splatter during key-up/key-down. Useful for CW and ASK/OOK shaping. |
| `0x2D` | `ASK_CFG` | ASK/OOK bandwidth/depth. | Only meaningful for ASK/OOK. OOK off state and depth are set here. |
| `0x2E` | `PKT_LEN` | Packet length or maximum packet length. | Fixed-length mode uses this as exact length; variable-length mode uses it as max length. |
| `0x2F` | Extended address marker | Prefix for extended-register access. | In this project, extended address `0x0C` is written as `0x2F0C`. |

### Extended Register Space

| Extended address | HT-15 address | Register | What it controls | Practical notes |
| ---: | ---: | --- | --- | --- |
| `0x00` | `0x2F00` | `IF_MIX_CFG` | Intermediate-frequency mixer choice. | Tied to RX bandwidth/decimation. Generated. |
| `0x01` | `0x2F01` | `FREQOFF_CFG` | Automatic frequency-offset correction. | Important for RX tolerance. Wider/longer-settling modes tolerate more offset. |
| `0x02` | `0x2F02` | `TOC_CFG` | Timing-offset correction. | Important for symbol timing recovery, especially with long packets or oscillator mismatch. |
| `0x03` | `0x2F03` | `MARC_SPARE` | AES helper command hooks and MARC spare fields. | Normally left alone unless using AES FIFO acceleration. |
| `0x04` | `0x2F04` | `ECG_CFG` | External clock generator divide setting. | Used when exporting clocks from the chip. |
| `0x05` | `0x2F05` | `MDMCFG2` | Extra modem options, including CFM enable. | Set `CFM_DATA_EN` for CW/CFM. TI notes CW requires this bit. |
| `0x06` | `0x2F06` | `EXT_CTRL` | External control and burst address behavior. | Includes pin-control enable and burst increment enable. Keep burst increment on for normal burst writes. |
| `0x07` | `0x2F07` | `RCCAL_FINE` | 40 kHz RC oscillator fine calibration. | Used by eWOR/low-power timing. Usually calibrated internally. |
| `0x08` | `0x2F08` | `RCCAL_COARSE` | 40 kHz RC oscillator coarse calibration. | Used by eWOR/low-power timing. |
| `0x09` | `0x2F09` | `RCCAL_OFFSET` | RC oscillator calibration offset. | Reserved/test-style field. Use generated/default values. |
| `0x0A` | `0x2F0A` | `FREQOFF1` | Frequency offset word high byte. | Signed offset applied to programmed frequency. Updated manually or by `SAFC`. |
| `0x0B` | `0x2F0B` | `FREQOFF0` | Frequency offset word low byte. | Use for crystal trim or AFC accumulation. |
| `0x0C` | `0x2F0C` | `FREQ2` | Frequency word high byte. | Carrier frequency bits `[23:16]`. |
| `0x0D` | `0x2F0D` | `FREQ1` | Frequency word middle byte. | Carrier frequency bits `[15:8]`. |
| `0x0E` | `0x2F0E` | `FREQ0` | Frequency word low byte. | Carrier frequency bits `[7:0]`. |
| `0x0F` | `0x2F0F` | `IF_ADC2` | IF ADC reserved/tuning fields. | Generated. |
| `0x10` | `0x2F10` | `IF_ADC1` | IF ADC reserved/tuning fields. | Generated. |
| `0x11` | `0x2F11` | `IF_ADC0` | IF ADC reserved/tuning fields. | Generated. |
| `0x12` | `0x2F12` | `FS_DIG1` | Frequency synthesizer digital tuning. | Generated. |
| `0x13` | `0x2F13` | `FS_DIG0` | Frequency synthesizer loop bandwidth settings. | Includes RX/TX PLL loop bandwidth fields. Generated unless optimizing lock behavior. |
| `0x14` | `0x2F14` | `FS_CAL3` | Frequency synthesizer calibration tuning. | Generated. |
| `0x15` | `0x2F15` | `FS_CAL2` | Frequency synthesizer calibration tuning. | Generated. |
| `0x16` | `0x2F16` | `FS_CAL1` | Frequency synthesizer calibration tuning. | Generated. |
| `0x17` | `0x2F17` | `FS_CAL0` | Synth lock detector averaging and calibration tuning. | Generated. |
| `0x18` | `0x2F18` | `FS_CHP` | Synthesizer charge-pump tuning. | Generated. |
| `0x19` | `0x2F19` | `FS_DIVTWO` | Synthesizer divide-by-two tuning. | Generated. |
| `0x1A` | `0x2F1A` | `FS_DSM1` | Synthesizer delta-sigma tuning. | Generated. |
| `0x1B` | `0x2F1B` | `FS_DSM0` | Synthesizer delta-sigma tuning. | Generated. |
| `0x1C` | `0x2F1C` | `FS_DVC1` | Synthesizer divider-chain tuning. | Generated. |
| `0x1D` | `0x2F1D` | `FS_DVC0` | Synthesizer divider-chain tuning. | Generated. |
| `0x1E` | `0x2F1E` | `FS_LBI` | Synthesizer local-bias/internal tuning. | Generated. |
| `0x1F` | `0x2F1F` | `FS_PFD` | Phase-frequency detector tuning. | Generated. |
| `0x20` | `0x2F20` | `FS_PRE` | Synthesizer prescaler tuning. | Generated. |
| `0x21` | `0x2F21` | `FS_REG_DIV_CML` | Synthesizer regulator/divider CML tuning. | Generated. |
| `0x22` | `0x2F22` | `FS_SPARE` | Synthesizer spare/internal tuning. | Generated. |
| `0x23` | `0x2F23` | `FS_VCO4` | VCO tuning. | Generated. |
| `0x24` | `0x2F24` | `FS_VCO3` | VCO tuning. | Generated. |
| `0x25` | `0x2F25` | `FS_VCO2` | VCO tuning. | Generated. |
| `0x26` | `0x2F26` | `FS_VCO1` | VCO calibration DAC and tuning. | Calibration may update/read this. Generated. |
| `0x27` | `0x2F27` | `FS_VCO0` | VCO tuning. | Generated. |
| `0x28` | `0x2F28` | `GBIAS6` | Global bias tuning. | Generated. |
| `0x29` | `0x2F29` | `GBIAS5` | Global bias tuning. | Generated. |
| `0x2A` | `0x2F2A` | `GBIAS4` | Global bias tuning. | Generated. |
| `0x2B` | `0x2F2B` | `GBIAS3` | Global bias tuning. | Generated. |
| `0x2C` | `0x2F2C` | `GBIAS2` | Global bias tuning. | Generated. |
| `0x2D` | `0x2F2D` | `GBIAS1` | Global bias tuning. | Generated. |
| `0x2E` | `0x2F2E` | `GBIAS0` | Global bias tuning. | Generated. |
| `0x2F` | `0x2F2F` | `IFAMP` | IF amplifier tuning. | Generated. |
| `0x30` | `0x2F30` | `LNA` | LNA tuning. | Generated. |
| `0x31` | `0x2F31` | `RXMIX` | RX mixer tuning. | Generated. |
| `0x32` | `0x2F32` | `XOSC5` | Crystal oscillator tuning. | Generated. |
| `0x33` | `0x2F33` | `XOSC4` | Crystal oscillator tuning. | Generated. |
| `0x34` | `0x2F34` | `XOSC3` | Crystal oscillator tuning. | Generated. |
| `0x35` | `0x2F35` | `XOSC2` | Crystal oscillator tuning and override behavior. | Generated unless explicitly forcing oscillator behavior. |
| `0x36` | `0x2F36` | `XOSC1` | Crystal oscillator buffer/stability fields. | Generated; stability can be read through status behavior. |
| `0x37` | `0x2F37` | `XOSC0` | Crystal oscillator tuning. | Generated. |
| `0x38` | `0x2F38` | `ANALOG_SPARE` | Analog spare/internal tuning. | Generated. |
| `0x39` | `0x2F39` | `PA_CFG3` | Additional PA configuration. | Generated/PA-specific. |
| `0x3A` to `0x3E` | `0x2F3A` to `0x2F3E` | Not used | No user function. | Do not write. |
| `0x3F` to `0x40` | `0x2F3F` to `0x2F40` | Reserved | Reserved. | Do not write. |
| `0x41` to `0x63` | `0x2F41` to `0x2F63` | Not used | No user function. | Do not write. |
| `0x64` | `0x2F64` | `WOR_TIME1` | Current eWOR timer high byte. | Status, not retained. |
| `0x65` | `0x2F65` | `WOR_TIME0` | Current eWOR timer low byte. | Status, not retained. |
| `0x66` | `0x2F66` | `WOR_CAPTURE1` | Captured eWOR timer high byte. | Status, not retained. |
| `0x67` | `0x2F67` | `WOR_CAPTURE0` | Captured eWOR timer low byte. | Status, not retained. |
| `0x68` | `0x2F68` | `BIST` | Built-in self-test/status. | Usually not used in normal firmware. |
| `0x69` | `0x2F69` | `DCFILTOFFSET_I1` | I-channel DC filter offset high byte. | RX diagnostic/status. |
| `0x6A` | `0x2F6A` | `DCFILTOFFSET_I0` | I-channel DC filter offset low byte. | RX diagnostic/status. |
| `0x6B` | `0x2F6B` | `DCFILTOFFSET_Q1` | Q-channel DC filter offset high byte. | RX diagnostic/status. |
| `0x6C` | `0x2F6C` | `DCFILTOFFSET_Q0` | Q-channel DC filter offset low byte. | RX diagnostic/status. |
| `0x6D` | `0x2F6D` | `IQIE_I1` | IQ image estimate I high byte. | RX diagnostic/status. |
| `0x6E` | `0x2F6E` | `IQIE_I0` | IQ image estimate I low byte. | RX diagnostic/status. |
| `0x6F` | `0x2F6F` | `IQIE_Q1` | IQ image estimate Q high byte. | RX diagnostic/status. |
| `0x70` | `0x2F70` | `IQIE_Q0` | IQ image estimate Q low byte. | RX diagnostic/status. |
| `0x71` | `0x2F71` | `RSSI1` | RSSI high/status byte. | Use with `RSSI0` to read measured signal strength. Useful for CW carrier detect. |
| `0x72` | `0x2F72` | `RSSI0` | RSSI low byte. | Use with `RSSI1`; conversion depends on mode and calibration. |
| `0x73` | `0x2F73` | `MARCSTATE` | Main radio control state. | Read to confirm IDLE/RX/TX/FIFO error states. |
| `0x74` | `0x2F74` | `LQI_VAL` | Link quality estimate. | Packet RX quality indicator; lower/higher meaning depends on TI interpretation and mode. |
| `0x75` | `0x2F75` | `PQT_SYNC_ERR` | Preamble/sync error counter/status. | Helps debug false sync or weak preamble. |
| `0x76` | `0x2F76` | `DEM_STATUS` | Demodulator status. | Use for RX debug and lock/acquisition visibility. |
| `0x77` | `0x2F77` | `FREQOFF_EST1` | Frequency offset estimate high byte. | Signed measured offset. Useful for AFC or calibration. |
| `0x78` | `0x2F78` | `FREQOFF_EST0` | Frequency offset estimate low byte. | Pairs with `FREQOFF_EST1`. |
| `0x79` | `0x2F79` | `AGC_GAIN3` | Current AGC front-end gain. | RX debug/status. |
| `0x7A` | `0x2F7A` | `AGC_GAIN2` | AGC gain override/status. | Can override AGC, but normal firmware leaves AGC in control. |
| `0x7B` | `0x2F7B` | `AGC_GAIN1` | AGC gain internal/status. | Debug/generated. |
| `0x7C` | `0x2F7C` | `AGC_GAIN0` | AGC gain internal/status. | Debug/generated. |
| `0x7D` | `0x2F7D` | `CFM_RX_DATA_OUT` | Signed CFM soft RX data. | Read repeatedly for custom FM demodulation. |
| `0x7E` | `0x2F7E` | `CFM_TX_DATA_IN` | Signed CFM soft TX data. | Write repeatedly for custom FM modulation. `0` gives center frequency. |
| `0x7F` | `0x2F7F` | `ASK_SOFT_RX_DATA` | ASK/OOK soft decision data. | Useful for custom ASK/OOK demodulation. |
| `0x80` | `0x2F80` | `RNDGEN` | Random number generator enable/value. | Can generate random bits; RX noise can further randomize. |
| `0x81` | `0x2F81` | `MAGN2` | Signal magnitude bit 16. | Instantaneous CORDIC magnitude diagnostic. |
| `0x82` | `0x2F82` | `MAGN1` | Signal magnitude bits 15:8. | Diagnostic. |
| `0x83` | `0x2F83` | `MAGN0` | Signal magnitude bits 7:0. | Diagnostic. |
| `0x84` | `0x2F84` | `ANG1` | Signal angle bits 9:8. | Instantaneous CORDIC angle diagnostic. |
| `0x85` | `0x2F85` | `ANG0` | Signal angle bits 7:0. | Diagnostic. |
| `0x86` | `0x2F86` | `CHFILT_I2` | Channel filter I data bit 16 and valid flag. | Raw channel-filter sample diagnostic. |
| `0x87` | `0x2F87` | `CHFILT_I1` | Channel filter I data bits 15:8. | Diagnostic. |
| `0x88` | `0x2F88` | `CHFILT_I0` | Channel filter I data bits 7:0. | Diagnostic. |
| `0x89` | `0x2F89` | `CHFILT_Q2` | Channel filter Q data bit 16 and valid flag. | Diagnostic. |
| `0x8A` | `0x2F8A` | `CHFILT_Q1` | Channel filter Q data bits 15:8. | Diagnostic. |
| `0x8B` | `0x2F8B` | `CHFILT_Q0` | Channel filter Q data bits 7:0. | Diagnostic. |
| `0x8C` | `0x2F8C` | `GPIO_STATUS` | GPIO status mirror. | Read current GDO/status pin states internally. |
| `0x8D` | `0x2F8D` | `FSCAL_CTRL` | Frequency synthesizer calibration control/status. | Calibration diagnostic/control. |
| `0x8E` | `0x2F8E` | `PHASE_ADJUST` | Phase adjustment/status. | Specialized calibration/debug. |
| `0x8F` | `0x2F8F` | `PARTNUMBER` | Chip part number. | Read during probe to confirm CC1200 family. |
| `0x90` | `0x2F90` | `PARTVERSION` | Chip revision/version. | Useful for diagnostics and errata handling. |
| `0x91` | `0x2F91` | `SERIAL_STATUS` | Serial/direct-access status and control. | Relevant to direct memory access and serial modes. |
| `0x92` | `0x2F92` | `MODEM_STATUS1` | Modem status flags. | Debug packet acquisition/demod state. |
| `0x93` | `0x2F93` | `MODEM_STATUS0` | Modem status flags. | Debug packet acquisition/demod state. |
| `0x94` | `0x2F94` | `MARC_STATUS1` | Radio control status/error flags. | Check after TX/RX failures. |
| `0x95` | `0x2F95` | `MARC_STATUS0` | Radio control status/error flags. | Check after TX/RX failures. |
| `0x96` | `0x2F96` | `PA_IFAMP_TEST` | PA/IF amp test register. | Test only. Use generated/default values. |
| `0x97` | `0x2F97` | `FSRF_TEST` | Frequency synthesizer/RF test register. | Test only. |
| `0x98` | `0x2F98` | `PRE_TEST` | Preamble/test register. | Test only. |
| `0x99` | `0x2F99` | `PRE_OVR` | Preamble override/test register. | Test only. |
| `0x9A` | `0x2F9A` | `ADC_TEST` | ADC test register. | Test only. |
| `0x9B` | `0x2F9B` | `DVC_TEST` | DVC test register. | Test only. |
| `0x9C` | `0x2F9C` | `ATEST` | Analog test register. | Test only. |
| `0x9D` | `0x2F9D` | `ATEST_LVDS` | LVDS analog test register. | Test only. |
| `0x9E` | `0x2F9E` | `ATEST_MODE` | Analog test mode. | Test only. |
| `0x9F` | `0x2F9F` | `XOSC_TEST1` | Crystal oscillator test register. | Test only. |
| `0xA0` | `0x2FA0` | `XOSC_TEST0` | Crystal oscillator test register. | Test only. |
| `0xA1` | `0x2FA1` | `AES` | AES engine control/status. | Used with AES key/workspace registers. |
| `0xA2` | `0x2FA2` | `MDM_TEST` | Modem test register. | Test only. |
| `0xA3` to `0xD1` | `0x2FA3` to `0x2FD1` | Not used | No user function. | Do not write. |
| `0xD2` | `0x2FD2` | `RXFIRST` | RX FIFO first pointer/status. | FIFO debug/status. |
| `0xD3` | `0x2FD3` | `TXFIRST` | TX FIFO first pointer/status. | FIFO debug/status. |
| `0xD4` | `0x2FD4` | `RXLAST` | RX FIFO last pointer/status. | FIFO debug/status. |
| `0xD5` | `0x2FD5` | `TXLAST` | TX FIFO last pointer/status. | FIFO debug/status. |
| `0xD6` | `0x2FD6` | `NUM_TXBYTES` | Number of bytes in TX FIFO/status. | Use before/after TX to detect underflow or drain state. |
| `0xD7` | `0x2FD7` | `NUM_RXBYTES` | Number of bytes in RX FIFO/status. | Use before reading packets. |
| `0xD8` | `0x2FD8` | `FIFO_NUM_TXBYTES` | TX FIFO byte count. | FIFO status. |
| `0xD9` | `0x2FD9` | `FIFO_NUM_RXBYTES` | RX FIFO byte count. | FIFO status. |
| `0xDA` | `0x2FDA` | `RXFIFO_PRE_BUF` | RX FIFO pre-buffer byte. | Allows access to the first byte in special direct-FIFO cases. |
| `0xDB` to `0xDF` | `0x2FDB` to `0x2FDF` | Not used | No user function. | Do not write. |
| `0xE0` to `0xEF` | `0x2FE0` to `0x2FEF` | `AES_KEY` | AES-128 key bytes. | Only needed when using hardware AES. |
| `0xE0` to `0xFF` | `0x2FE0` to `0x2FFF` | AES workspace | AES engine workspace/buffer overlap region. | Treat as owned by AES engine when AES is enabled. |
| `0xF0` to `0xFF` | `0x2FF0` to `0x2FFF` | `AES_BUFFER` | AES input/output buffer bytes. | Only needed when using hardware AES. |

## Common Bring-Up Sequence

Use this sequence before mode-specific TX/RX:

```c
static void cc1200_idle_and_flush(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_write_cmd(dev, SIDLE);
    sleep_ms(1);
    rfmodule_2m70cm_write_cmd(dev, SFRX);
    rfmodule_2m70cm_write_cmd(dev, SFTX);
}

static void cc1200_set_frequency_146m(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_write_register(dev, 0x20, 0x0B);   /* FS_CFG, 136.7-160 MHz band */
    rfmodule_2m70cm_write_register(dev, 0x2F0C, 0x57); /* FREQ2 */
    rfmodule_2m70cm_write_register(dev, 0x2F0D, 0x99); /* FREQ1 */
    rfmodule_2m70cm_write_register(dev, 0x2F0E, 0x9A); /* FREQ0 */
}

static void cc1200_calibrate(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_write_cmd(dev, SIDLE);
    sleep_ms(1);
    rfmodule_2m70cm_write_cmd(dev, SCAL);
    sleep_ms(10);
}
```

For a real link, also load the generated SmartRF Studio settings for:

- RX channel bandwidth: `CHAN_BW`
- AGC: `AGC_REF`, `AGC_CS_THR`, `AGC_CFG3..0`
- Packet format: `SYNCx`, `SYNC_CFGx`, `PREAMBLE_CFGx`, `PKT_CFGx`, `PKT_LEN`
- PA power/ramp: `PA_CFG1`, `PA_CFG0`, possibly `PA_CFG3`
- Synth/analog tuning: `IF_MIX_CFG`, `FS_*`, `GBIAS*`, `XOSC*`, `LNA`, `RXMIX`

## CW TX and RX

### CW TX: key an unmodulated carrier

TI's CW guidance is to enable CFM data mode with `MDMCFG2.CFM_DATA_EN = 1`.
Writing `0` to `CFM_TX_DATA_IN` keeps the carrier centered. Then `STX` starts
the carrier. To key off, strobe `SIDLE` or power down the RF path.

```c
static void cc1200_cw_tx_on_146m(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_ON);
    cc1200_idle_and_flush(dev);
    cc1200_set_frequency_146m(dev);

    rfmodule_2m70cm_write_register(dev, 0x2F05, 0x01); /* MDMCFG2.CFM_DATA_EN */
    rfmodule_2m70cm_write_register(dev, 0x2F7E, 0x00); /* CFM_TX_DATA_IN center */

    cc1200_calibrate(dev);
    rfmodule_2m70cm_write_cmd(dev, STX);
}

static void cc1200_cw_tx_off(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_write_cmd(dev, SIDLE);
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_OFF);
}
```

For Morse, call `cc1200_cw_tx_on_146m()` for marks and
`cc1200_cw_tx_off()` for spaces. For clean spectral behavior, prefer leaving
the CC1200 configured and only transitioning between `STX` and `SIDLE`, with
reasonable PA ramping in `PA_CFG1/0`.

### CW RX: detect carrier

The CC1200 does not decode Morse timing for you. A simple CW receiver path uses
RSSI/carrier-sense as an envelope detector, then firmware turns mark/space
timing into dots and dashes.

```c
static void cc1200_cw_rx_start_146m(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_RX_ONLY);
    cc1200_idle_and_flush(dev);
    cc1200_set_frequency_146m(dev);

    /* Configure RX bandwidth and AGC from SmartRF Studio before SRX. */
    cc1200_calibrate(dev);
    rfmodule_2m70cm_write_cmd(dev, SRX);
}

static bool8 cc1200_cw_carrier_present(rfmodule_config_t *dev)
{
    u8 rssi1 = rfmodule_2m70cm_read_register(dev, 0x2F71);
    u8 rssi0 = rfmodule_2m70cm_read_register(dev, 0x2F72);
    (void)rssi0;

    /*
     * Replace this with a calibrated RSSI conversion/threshold for the board.
     * You can also configure a GPIO for carrier-sense and read the pin.
     */
    return (rssi1 > 0x40);
}
```

For better CW receive, read `CFM_RX_DATA_OUT` as a signed frequency-offset
sample and combine it with RSSI. RSSI tells you a carrier exists; CFM soft data
helps show whether the carrier is centered or offset.

## Custom FM TX and RX Through CFM

CFM is the way to make the CC1200 do analog-like FM controlled by firmware.
`CFM_TX_DATA_IN` is a signed frequency-offset sample. `0` is center frequency.
Positive values shift one direction; negative values shift the other. The
deviation register sets the scale.

Approximate offset:

```text
f_offset_hz = signed_cfm_sample * f_dev_hz / 64
```

### CFM FM TX

This example assumes you already have signed audio samples scaled into about
`-64` to `+63`. You must write samples at a stable rate. For voice-like FM, that
means a timer/DMA/SPI path, not occasional writes from the UI loop.

```c
static void cc1200_fm_tx_start_146m(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_ON);
    cc1200_idle_and_flush(dev);
    cc1200_set_frequency_146m(dev);

    /* Example: about 5 kHz peak deviation with 40 MHz XOSC. */
    rfmodule_2m70cm_write_register(dev, 0x0A, 0x06); /* DEVIATION_M */
    rfmodule_2m70cm_write_register(dev, 0x0B, 0x01); /* MODCFG_DEV_E: 2-FSK bits + DEV_E=1 */

    rfmodule_2m70cm_write_register(dev, 0x2F05, 0x01); /* MDMCFG2.CFM_DATA_EN */
    rfmodule_2m70cm_write_register(dev, 0x2F7E, 0x00); /* Start centered */

    cc1200_calibrate(dev);
    rfmodule_2m70cm_write_cmd(dev, STX);
}

static void cc1200_fm_tx_sample(rfmodule_config_t *dev, i8 sample)
{
    if(sample > 63) sample = 63;
    if(sample < -64) sample = -64;
    rfmodule_2m70cm_write_register(dev, 0x2F7E, (u8)sample);
}
```

### CFM FM RX

In CFM RX, firmware reads signed soft frequency samples from
`CFM_RX_DATA_OUT`. This is not a complete audio receiver by itself: you still
need sample timing, de-emphasis/filtering, squelch, and audio output.

```c
static void cc1200_fm_rx_start_146m(rfmodule_config_t *dev)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_RX_ONLY);
    cc1200_idle_and_flush(dev);
    cc1200_set_frequency_146m(dev);

    /* Use matching deviation and RX bandwidth settings. */
    rfmodule_2m70cm_write_register(dev, 0x0A, 0x06); /* DEVIATION_M */
    rfmodule_2m70cm_write_register(dev, 0x0B, 0x01); /* DEV_E=1 */
    rfmodule_2m70cm_write_register(dev, 0x2F05, 0x01); /* CFM_DATA_EN */

    cc1200_calibrate(dev);
    rfmodule_2m70cm_write_cmd(dev, SRX);
}

static i8 cc1200_fm_rx_sample(rfmodule_config_t *dev)
{
    return (i8)rfmodule_2m70cm_read_register(dev, 0x2F7D);
}
```

## 2-FSK / Packet FM TX and RX

This is the normal packet mode most people mean by "FM data" on CC1200. Use
`MODCFG_DEV_E.MOD_FORMAT = 000` for 2-FSK or `001` for 2-GFSK.

Example target:

- Carrier: 146.000 MHz
- Bit rate: 9600 bit/s
- Symbol rate: 9600 sym/s
- Deviation: about 5 kHz
- Packet mode: fixed length in this example

Core modem values for the example:

```text
DEVIATION_M = 0x06
MODCFG_DEV_E = 0x01       ; 2-FSK, DEV_E=1
SYMBOL_RATE2/1/0 = 0x6F, 0x75, 0x10
```

### 2-FSK TX

```c
static void cc1200_2fsk_packet_config_9600(rfmodule_config_t *dev, u8 packet_len)
{
    cc1200_set_frequency_146m(dev);

    rfmodule_2m70cm_write_register(dev, 0x0A, 0x06); /* DEVIATION_M, about 5 kHz */
    rfmodule_2m70cm_write_register(dev, 0x0B, 0x01); /* MODCFG_DEV_E: 2-FSK, DEV_E=1 */
    rfmodule_2m70cm_write_register(dev, 0x13, 0x6F); /* SYMBOL_RATE2 */
    rfmodule_2m70cm_write_register(dev, 0x14, 0x75); /* SYMBOL_RATE1 */
    rfmodule_2m70cm_write_register(dev, 0x15, 0x10); /* SYMBOL_RATE0 */

    /*
     * Load SmartRF Studio values here for CHAN_BW, MDMCFGx, AGC_CFGx,
     * SYNC_CFGx, PREAMBLE_CFGx, PKT_CFGx, RFEND_CFGx, and PA_CFGx.
     */
    rfmodule_2m70cm_write_register(dev, 0x2E, packet_len); /* PKT_LEN */
}

static void cc1200_2fsk_tx_packet(rfmodule_config_t *dev, const u8 *data, u8 len)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_ON);
    cc1200_idle_and_flush(dev);
    cc1200_2fsk_packet_config_9600(dev, len);
    cc1200_calibrate(dev);

    cc1200_write_fifo(dev, data, len); /* Implement as burst write to 0x3F. */
    rfmodule_2m70cm_write_cmd(dev, STX);

    /* Poll MARCSTATE, NUM_TXBYTES, or a GPIO packet-done signal here. */
}
```

### 2-FSK RX

```c
static void cc1200_2fsk_rx_start(rfmodule_config_t *dev, u8 packet_len)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_RX_ONLY);
    cc1200_idle_and_flush(dev);
    cc1200_2fsk_packet_config_9600(dev, packet_len);
    cc1200_calibrate(dev);
    rfmodule_2m70cm_write_cmd(dev, SRX);
}

static u8 cc1200_2fsk_rx_poll(rfmodule_config_t *dev, u8 *out, u8 max_len)
{
    u8 bytes = rfmodule_2m70cm_read_register(dev, 0x2FD7); /* NUM_RXBYTES */
    if(bytes == 0) return 0;
    if(bytes > max_len) bytes = max_len;

    cc1200_read_fifo(dev, out, bytes); /* Implement as burst read from 0x3F. */
    return bytes;
}
```

## 4-FSK TX and RX

4-FSK uses four tones, carrying two bits per symbol. That means:

```text
bit_rate = 2 * symbol_rate
symbol_rate = bit_rate / 2
```

Use `MODCFG_DEV_E.MOD_FORMAT = 100` for 4-FSK or `101` for 4-GFSK.

Example target:

- Carrier: 146.000 MHz
- Bit rate: 9600 bit/s
- Symbol rate: 4800 sym/s
- Deviation: about 5 kHz nominal scale
- Packet mode: fixed length in this example

Core modem values for the example:

```text
DEVIATION_M = 0x06
MODCFG_DEV_E = 0x21       ; 4-FSK, DEV_E=1
SYMBOL_RATE2/1/0 = 0x5F, 0x75, 0x10
```

### 4-FSK TX

```c
static void cc1200_4fsk_packet_config_9600(rfmodule_config_t *dev, u8 packet_len)
{
    cc1200_set_frequency_146m(dev);

    rfmodule_2m70cm_write_register(dev, 0x0A, 0x06); /* DEVIATION_M */
    rfmodule_2m70cm_write_register(dev, 0x0B, 0x21); /* MODCFG_DEV_E: 4-FSK, DEV_E=1 */
    rfmodule_2m70cm_write_register(dev, 0x13, 0x5F); /* 4800 sym/s */
    rfmodule_2m70cm_write_register(dev, 0x14, 0x75);
    rfmodule_2m70cm_write_register(dev, 0x15, 0x10);

    /*
     * Load SmartRF Studio values for the rest of the modem/filter/AGC/packet
     * registers. 4-FSK is less forgiving than 2-FSK if RX bandwidth, deviation,
     * and timing loops are mismatched.
     */
    rfmodule_2m70cm_write_register(dev, 0x2E, packet_len);
}

static void cc1200_4fsk_tx_packet(rfmodule_config_t *dev, const u8 *data, u8 len)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_ON);
    cc1200_idle_and_flush(dev);
    cc1200_4fsk_packet_config_9600(dev, len);
    cc1200_calibrate(dev);

    cc1200_write_fifo(dev, data, len);
    rfmodule_2m70cm_write_cmd(dev, STX);
}
```

### 4-FSK RX

```c
static void cc1200_4fsk_rx_start(rfmodule_config_t *dev, u8 packet_len)
{
    rfmodule_2m70cm_set_power_mode(dev, RFMODULE_2M70CM_POWER_MODE_RX_ONLY);
    cc1200_idle_and_flush(dev);
    cc1200_4fsk_packet_config_9600(dev, packet_len);
    cc1200_calibrate(dev);
    rfmodule_2m70cm_write_cmd(dev, SRX);
}

static u8 cc1200_4fsk_rx_poll(rfmodule_config_t *dev, u8 *out, u8 max_len)
{
    u8 bytes = rfmodule_2m70cm_read_register(dev, 0x2FD7); /* NUM_RXBYTES */
    if(bytes == 0) return 0;
    if(bytes > max_len) bytes = max_len;

    cc1200_read_fifo(dev, out, bytes);
    return bytes;
}
```

## Practical Debug Checklist

1. Confirm SPI first. Read `PARTNUMBER` (`0x2F8F`) and `PARTVERSION`
   (`0x2F90`) before testing RF.
2. Confirm GPIO/power routing. On the HT-15, module GPIO pins also control reset
   and RF amp power, so board-level naming matters.
3. Use `SIDLE` before changing frequency, modulation, or synthesizer registers.
4. Program `FS_CFG` before `FREQ2/1/0`.
5. Calibrate after frequency/band changes and before entering `STX` or `SRX`.
6. For CW, set `MDMCFG2.CFM_DATA_EN` and write `CFM_TX_DATA_IN = 0`.
7. For packet modes, do not skip RX bandwidth and AGC settings. Frequency,
   deviation, and symbol rate alone are not a complete modem configuration.
8. Use `MARCSTATE`, `MARC_STATUSx`, `NUM_TXBYTES`, and `NUM_RXBYTES` to debug
   stuck TX/RX, FIFO underflow, FIFO overflow, and packet acquisition failures.
9. If packets do not receive, verify both sides match: frequency, modulation,
   deviation, symbol rate, RX bandwidth, preamble, sync word, packet length mode,
   CRC, address filtering, and whitening/FEC/Manchester settings.
