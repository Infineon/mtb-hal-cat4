# CAT4 (4390X) Hardware Abstraction Layer (HAL) Release Notes
The CAT4 Hardware Abstraction Layer (HAL) provides an implementation of the Hardware Abstraction Layer for the 4390X chip families. This API provides convenience methods for initializing and manipulating different hardware peripherals. Depending on the specific chip being used, not all features may be supported.

### What's Included?
This release of the CAT4 HAL includes support for the following drivers:
* Clock
* DMA
* Memory to Memory DMA (M2M DMA)
* GPIO
* Hardware Manager
* I2C
* LowPower Timer (LPTimer)
* Power Management (SysPM)
* PWM
* SPI
* System
* Timer
* UART
* WDT

The following drivers will be supported in a future release:
* QSPI

The following HAL drivers are not supported on CAT4 as none of the devices in ModusToolbox™ have the necessary hardware support:
* ADC
* Comparator
* CRC
* DAC
* EZ-I2C
* Flash
* I2S
* OpAmp
* PDM/PCM
* Quadrature Decoder (QuadDec)
* RTC
* SDHC
* SDIO
* TDM
* True Random Number Generator (TRNG)
* USB Device

### What Changed?
#### v1.0.1
* Fix failure to retrieve clock frequency when exiting low power modes
* Rename `wiced_result_t` in UART ringbuffer to avoid naming collision with other libraries.
#### v1.0.0
* Initial production release
#### v0.5.0
* Initial release supporting GCC_ARM

### Supported Software and Tools
This version of the CAT4 Hardware Abstraction Layer was validated for compatibility with the following Software and Tools:

| Software and Tools                        | Version |
| :---                                      | :----:  |
| ModusToolbox™ Software Environment        | 3.0.0   |
| GCC Compiler                              | 10.3.1  |

Minimum required ModusToolbox™ Software Environment: v3.0.0

### More information
Use the following links for more information, as needed:
* [API Reference Guide](https://infineon.github.io/mtb-hal-cat2/html/modules.html)
* [Cypress Semiconductor, an Infineon Technologies Company](http://www.cypress.com)
* [Infineon GitHub](https://github.com/infineon)
* [ModusToolbox™](https://www.cypress.com/products/modustoolbox-software-environment)

---
© Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation, 2019-2023.
