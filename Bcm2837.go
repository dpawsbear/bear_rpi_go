//package bear_rpi_go
package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"os"
	"syscall"
	"time"
	"unsafe"
)

/* Bcm2837.go

   go support for Broadcom BCM 2837 as used in Raspberry Pi

   Author: dpawsbear
   Copyright (C) 2017-2018 dpawsbear

   changelog :
   		2017.08.30 dpawsbear : first create
   		2017.08.31 dpawsbear : add interrupt function
   		2017.08.31 dpawsbear : ready for package
   		2017.09.01 dpawsbear : start to write i2c drivers (not test)

*/

const (
	err             = 55
	BCM2837_VERSION = 10001

	// define the pin level
	HIGH = 0x1
	LOW  = 0x0

	// define the clock
	BCM2837_CORE_CLK_HZ = 111 //TODO not ok

	// define the device tree range
	BCM2837_PRI3B_DT_FILENAME                = "/proc/device-tree/soc/ranges"
	BCM2837_PRI3_DT_PERI_BASE_ADDRESS_OFFSET = 0x4
	BCM2837_PRI3_DT_PERI_SIZE_OFFSET         = 0X8

	// define Peripherals block base address on Rpi
	BCM2837_PERI_BASE uint32 = 0x3f000000
	BCM2837_PERI_SIZE uint32 = 0x01000000

	//define the base of peripherals block
	BCM2837_ST_BASE    uint32 = 0x003000
	BCM2837_GPIO_PADS  uint32 = 0x100000
	BCM2837_CLOCK_BASE uint32 = 0x101000
	BCM2837_GPIO_BASE  uint32 = 0x200000
	BCM2837_SPI0_BASE  uint32 = 0x204000
	BCM2837_BSC0_BASE  uint32 = 0x205000
	BCM2837_GPIO_PWM   uint32 = 0x20C000
	BCM2837_BSC1_BASE  uint32 = 0x804000

	// register base
	BCM2837_REGBASE_ST   = 1 /*!< Base of the ST (System Timer) registers. */
	BCM2837_REGBASE_GPIO = 2 /*!< Base of the GPIO registers. */
	BCM2837_REGBASE_PWM  = 3 /*!< Base of the PWM registers. */
	BCM2837_REGBASE_CLK  = 4 /*!< Base of the CLK registers. */
	BCM2837_REGBASE_PADS = 5 /*!< Base of the PADS registers. */
	BCM2837_REGBASE_SPI0 = 6 /*!< Base of the SPI0 registers. */
	BCM2837_REGBASE_BSC0 = 7 /*!< Base of the BSC0 registers. */
	BCM2837_REGBASE_BSC1 = 8 /*!< Base of the BSC1 registers. */

	//Memory page and block
	BCM2837_PAGE_SIZE  = (4 * 1024)
	BCM2837_BLOCK_SIZE = (4 * 1024)

	/*! GPIO register offsets from BCM2835_GPIO_BASE.
	Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
	*/
	BCM2837_GPFSEL0   uint32 = 0x0000 /*!< GPIO Function Select 0 */
	BCM2837_GPFSEL1   uint32 = 0x0004 /*!< GPIO Function Select 1 */
	BCM2837_GPFSEL2   uint32 = 0x0008 /*!< GPIO Function Select 2 */
	BCM2837_GPFSEL3   uint32 = 0x000c /*!< GPIO Function Select 3 */
	BCM2837_GPFSEL4   uint32 = 0x0010 /*!< GPIO Function Select 4 */
	BCM2837_GPFSEL5   uint32 = 0x0014 /*!< GPIO Function Select 5 */
	BCM2837_GPSET0    uint32 = 0x001c /*!< GPIO Pin Output Set 0 */
	BCM2837_GPSET1    uint32 = 0x0020 /*!< GPIO Pin Output Set 1 */
	BCM2837_GPCLR0    uint32 = 0x0028 /*!< GPIO Pin Output Clear 0 */
	BCM2837_GPCLR1    uint32 = 0x002c /*!< GPIO Pin Output Clear 1 */
	BCM2837_GPLEV0    uint32 = 0x0034 /*!< GPIO Pin Level 0 */
	BCM2837_GPLEV1    uint32 = 0x0038 /*!< GPIO Pin Level 1 */
	BCM2837_GPEDS0    uint32 = 0x0040 /*!< GPIO Pin Event Detect Status 0 */
	BCM2837_GPEDS1    uint32 = 0x0044 /*!< GPIO Pin Event Detect Status 1 */
	BCM2837_GPREN0    uint32 = 0x004c /*!< GPIO Pin Rising Edge Detect Enable 0 */
	BCM2837_GPREN1    uint32 = 0x0050 /*!< GPIO Pin Rising Edge Detect Enable 1 */
	BCM2837_GPFEN0    uint32 = 0x0058 /*!< GPIO Pin Falling Edge Detect Enable 0 */
	BCM2837_GPFEN1    uint32 = 0x005c /*!< GPIO Pin Falling Edge Detect Enable 1 */
	BCM2837_GPHEN0    uint32 = 0x0064 /*!< GPIO Pin High Detect Enable 0 */
	BCM2837_GPHEN1    uint32 = 0x0068 /*!< GPIO Pin High Detect Enable 1 */
	BCM2837_GPLEN0    uint32 = 0x0070 /*!< GPIO Pin Low Detect Enable 0 */
	BCM2837_GPLEN1    uint32 = 0x0074 /*!< GPIO Pin Low Detect Enable 1 */
	BCM2837_GPAREN0   uint32 = 0x007c /*!< GPIO Pin Async. Rising Edge Detect 0 */
	BCM2837_GPAREN1   uint32 = 0x0080 /*!< GPIO Pin Async. Rising Edge Detect 1 */
	BCM2837_GPAFEN0   uint32 = 0x0088 /*!< GPIO Pin Async. Falling Edge Detect 0 */
	BCM2837_GPAFEN1   uint32 = 0x008c /*!< GPIO Pin Async. Falling Edge Detect 1 */
	BCM2837_GPPUD     uint32 = 0x0094 /*!< GPIO Pin Pull-up/down Enable */
	BCM2837_GPPUDCLK0 uint32 = 0x0098 /*!< GPIO Pin Pull-up/down Enable Clock 0 */
	BCM2837_GPPUDCLK1 uint32 = 0x009c /*!< GPIO Pin Pull-up/down Enable Clock 1 */

	//define gpio function select
	BCM2837_GPIO_FSEL_INPT        = 0x00 /*!< Input 0b000 */
	BCM2837_GPIO_FSEL_OUTP        = 0x01 /*!< Output 0b001 */
	BCM2837_GPIO_FSEL_ALT0        = 0x04 /*!< Alternate function 0 0b100 */
	BCM2837_GPIO_FSEL_ALT1        = 0x05 /*!< Alternate function 1 0b101 */
	BCM2837_GPIO_FSEL_ALT2        = 0x06 /*!< Alternate function 2 0b110, */
	BCM2837_GPIO_FSEL_ALT3        = 0x07 /*!< Alternate function 3 0b111 */
	BCM2837_GPIO_FSEL_ALT4        = 0x03 /*!< Alternate function 4 0b011 */
	BCM2837_GPIO_FSEL_ALT5 uint32 = 0x02 /*!< Alternate function 5 0b010 */
	BCM2837_GPIO_FSEL_MASK uint32 = 0x07 /*!< Function select bits mask 0b111 */

	//gpio PUP control
	BCM2837_GPIO_PUD_OFF  = 0x00 /*!< Off ? disable pull-up/down 0b00 */
	BCM2837_GPIO_PUD_DOWN = 0x01 /*!< Enable Pull Down control 0b01 */
	BCM2837_GPIO_PUD_UP   = 0x02 /*!< Enable Pull Up control 0b10  */

	/*! Pad control register offsets from BCM2835_GPIO_PADS */
	BCM2837_PADS_GPIO_0_27  = 0x002c /*!< Pad control register for pads 0 to 27 */
	BCM2837_PADS_GPIO_28_45 = 0x0030 /*!< Pad control register for pads 28 to 45 */
	BCM2837_PADS_GPIO_46_53 = 0x0034 /*!< Pad control register for pads 46 to 53 */

	/*! Pad Control masks */
	BCM2837_PAD_PASSWRD             = (0x5A << 24) /*!< Password to enable setting pad mask */
	BCM2837_PAD_SLEW_RATE_UNLIMITED = 0x10         /*!< Slew rate unlimited */
	BCM2837_PAD_HYSTERESIS_ENABLED  = 0x08         /*!< Hysteresis enabled */
	BCM2837_PAD_DRIVE_2mA           = 0x00         /*!< 2mA drive current */
	BCM2837_PAD_DRIVE_4mA           = 0x01         /*!< 4mA drive current */
	BCM2837_PAD_DRIVE_6mA           = 0x02         /*!< 6mA drive current */
	BCM2837_PAD_DRIVE_8mA           = 0x03         /*!< 8mA drive current */
	BCM2837_PAD_DRIVE_10mA          = 0x04         /*!< 10mA drive current */
	BCM2837_PAD_DRIVE_12mA          = 0x05         /*!< 12mA drive current */
	BCM2837_PAD_DRIVE_14mA          = 0x06         /*!< 14mA drive current */
	BCM2837_PAD_DRIVE_16mA          = 0x07         /*!< 16mA drive current */

	//bcm 2837 pad group
	BCM2837_PAD_GROUP_GPIO_0_27  = 0 /*!< Pad group for GPIO pads 0 to 27 */
	BCM2837_PAD_GROUP_GPIO_28_45 = 1 /*!< Pad group for GPIO pads 28 to 45 */
	BCM2837_PAD_GROUP_GPIO_46_53 = 2 /*!< Pad group for GPIO pads 46 to 53 */

	// GPIO pin number
	RPI_3B_GPIO_J8_03 = 2  /*!< B+, Pin J8-03 */
	RPI_3B_GPIO_J8_05 = 3  /*!< B+, Pin J8-05 */
	RPI_3B_GPIO_J8_07 = 4  /*!< B+, Pin J8-07 */
	RPI_3B_GPIO_J8_08 = 14 /*!< B+, Pin J8-08, defaults to alt function 0 UART0_TXD */
	RPI_3B_GPIO_J8_10 = 15 /*!< B+, Pin J8-10, defaults to alt function 0 UART0_RXD */
	RPI_3B_GPIO_J8_11 = 17 /*!< B+, Pin J8-11 */
	RPI_3B_GPIO_J8_12 = 18 /*!< B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5 */
	RPI_3B_GPIO_J8_13 = 27 /*!< B+, Pin J8-13 */
	RPI_3B_GPIO_J8_15 = 22 /*!< B+, Pin J8-15 */
	RPI_3B_GPIO_J8_16 = 23 /*!< B+, Pin J8-16 */
	RPI_3B_GPIO_J8_18 = 24 /*!< B+, Pin J8-18 */
	RPI_3B_GPIO_J8_19 = 10 /*!< B+, Pin J8-19, MOSI when SPI0 in use */
	RPI_3B_GPIO_J8_21 = 9  /*!< B+, Pin J8-21, MISO when SPI0 in use */
	RPI_3B_GPIO_J8_22 = 25 /*!< B+, Pin J8-22 */
	RPI_3B_GPIO_J8_23 = 11 /*!< B+, Pin J8-23, CLK when SPI0 in use */
	RPI_3B_GPIO_J8_24 = 8  /*!< B+, Pin J8-24, CE0 when SPI0 in use */
	RPI_3B_GPIO_J8_26 = 7  /*!< B+, Pin J8-26, CE1 when SPI0 in use */
	RPI_3B_GPIO_J8_29 = 5  /*!< B+, Pin J8-29,  */
	RPI_3B_GPIO_J8_31 = 6  /*!< B+, Pin J8-31,  */
	RPI_3B_GPIO_J8_32 = 12 /*!< B+, Pin J8-32,  */
	RPI_3B_GPIO_J8_33 = 13 /*!< B+, Pin J8-33,  */
	RPI_3B_GPIO_J8_35 = 19 /*!< B+, Pin J8-35,  */
	RPI_3B_GPIO_J8_36 = 16 /*!< B+, Pin J8-36,  */
	RPI_3B_GPIO_J8_37 = 26 /*!< B+, Pin J8-37,  */
	RPI_3B_GPIO_J8_38 = 20 /*!< B+, Pin J8-38,  */
	RPI_3B_GPIO_J8_40 = 21 /*!< B+, Pin J8-40,  */

	//define for spi
	BCM2837_SPI0_CS   = 0x0000 /*!< SPI Master Control and Status */
	BCM2837_SPI0_FIFO = 0x0004 /*!< SPI Master TX and RX FIFOs */
	BCM2837_SPI0_CLK  = 0x0008 /*!< SPI Master Clock Divider */
	BCM2837_SPI0_DLEN = 0x000c /*!< SPI Master Data Length */
	BCM2837_SPI0_LTOH = 0x0010 /*!< SPI LOSSI mode TOH */
	BCM2837_SPI0_DC   = 0x0014 /*!< SPI DMA DREQ Controls */

	/* Register masks for SPI0_CS */
	BCM2837_SPI0_CS_LEN_LONG = 0x02000000 /*!< Enable Long data word in Lossi mode if DMA_LEN is set */
	BCM2837_SPI0_CS_DMA_LEN  = 0x01000000 /*!< Enable DMA mode in Lossi mode */
	BCM2837_SPI0_CS_CSPOL2   = 0x00800000 /*!< Chip Select 2 Polarity */
	BCM2837_SPI0_CS_CSPOL1   = 0x00400000 /*!< Chip Select 1 Polarity */
	BCM2837_SPI0_CS_CSPOL0   = 0x00200000 /*!< Chip Select 0 Polarity */
	BCM2837_SPI0_CS_RXF      = 0x00100000 /*!< RXF - RX FIFO Full */
	BCM2837_SPI0_CS_RXR      = 0x00080000 /*!< RXR RX FIFO needs Reading (full) */
	BCM2837_SPI0_CS_TXD      = 0x00040000 /*!< TXD TX FIFO can accept Data */
	BCM2837_SPI0_CS_RXD      = 0x00020000 /*!< RXD RX FIFO contains Data */
	BCM2837_SPI0_CS_DONE     = 0x00010000 /*!< Done transfer Done */
	BCM2837_SPI0_CS_TE_EN    = 0x00008000 /*!< Unused */
	BCM2837_SPI0_CS_LMONO    = 0x00004000 /*!< Unused */
	BCM2837_SPI0_CS_LEN      = 0x00002000 /*!< LEN LoSSI enable */
	BCM2837_SPI0_CS_REN      = 0x00001000 /*!< REN Read Enable */
	BCM2837_SPI0_CS_ADCS     = 0x00000800 /*!< ADCS Automatically Deassert Chip Select */
	BCM2837_SPI0_CS_INTR     = 0x00000400 /*!< INTR Interrupt on RXR */
	BCM2837_SPI0_CS_INTD     = 0x00000200 /*!< INTD Interrupt on Done */
	BCM2837_SPI0_CS_DMAEN    = 0x00000100 /*!< DMAEN DMA Enable */
	BCM2837_SPI0_CS_TA       = 0x00000080 /*!< Transfer Active */
	BCM2837_SPI0_CS_CSPOL    = 0x00000040 /*!< Chip Select Polarity */
	BCM2837_SPI0_CS_CLEAR    = 0x00000030 /*!< Clear FIFO Clear RX and TX */
	BCM2837_SPI0_CS_CLEAR_RX = 0x00000020 /*!< Clear FIFO Clear RX  */
	BCM2837_SPI0_CS_CLEAR_TX = 0x00000010 /*!< Clear FIFO Clear TX  */
	BCM2837_SPI0_CS_CPOL     = 0x00000008 /*!< Clock Polarity */
	BCM2837_SPI0_CS_CPHA     = 0x00000004 /*!< Clock Phase */
	BCM2837_SPI0_CS_CS       = 0x00000003 /*!< Chip Select */

	// set spi bit order
	BCM2837_SPI_BIT_ORDER_LSBFIRST = 0 /*!< LSB First */
	BCM2837_SPI_BIT_ORDER_MSBFIRST = 1 /*!< MSB First */

	//spi set data mode
	BCM2837_SPI_MODE0 = 0 /*!< CPOL = 0, CPHA = 0 */
	BCM2837_SPI_MODE1 = 1 /*!< CPOL = 0, CPHA = 1 */
	BCM2837_SPI_MODE2 = 2 /*!< CPOL = 1, CPHA = 0 */
	BCM2837_SPI_MODE3 = 3 /*!< CPOL = 1, CPHA = 1 */

	//SPI chip select
	BCM2837_SPI_CS0     = 0 /*!< Chip Select 0 */
	BCM2837_SPI_CS1     = 1 /*!< Chip Select 1 */
	BCM2837_SPI_CS2     = 2 /*!< Chip Select 2 (ie pins CS1 and CS2 are asserted) */
	BCM2837_SPI_CS_NONE = 3 /*!< No CS, control it yourself */

	//spi clock divider
	BCM2837_SPI_CLOCK_DIVIDER_65536 = 0     /*!< 65536 = 262.144us = 3.814697260kHz */
	BCM2837_SPI_CLOCK_DIVIDER_32768 = 32768 /*!< 32768 = 131.072us = 7.629394531kHz */
	BCM2837_SPI_CLOCK_DIVIDER_16384 = 16384 /*!< 16384 = 65.536us = 15.25878906kHz */
	BCM2837_SPI_CLOCK_DIVIDER_8192  = 8192  /*!< 8192 = 32.768us = 30/51757813kHz */
	BCM2837_SPI_CLOCK_DIVIDER_4096  = 4096  /*!< 4096 = 16.384us = 61.03515625kHz */
	BCM2837_SPI_CLOCK_DIVIDER_2048  = 2048  /*!< 2048 = 8.192us = 122.0703125kHz */
	BCM2837_SPI_CLOCK_DIVIDER_1024  = 1024  /*!< 1024 = 4.096us = 244.140625kHz */
	BCM2837_SPI_CLOCK_DIVIDER_512   = 512   /*!< 512 = 2.048us = 488.28125kHz */
	BCM2837_SPI_CLOCK_DIVIDER_256   = 256   /*!< 256 = 1.024us = 976.5625kHz */
	BCM2837_SPI_CLOCK_DIVIDER_128   = 128   /*!< 128 = 512ns = = 1.953125MHz */
	BCM2837_SPI_CLOCK_DIVIDER_64    = 64    /*!< 64 = 256ns = 3.90625MHz */
	BCM2837_SPI_CLOCK_DIVIDER_32    = 32    /*!< 32 = 128ns = 7.8125MHz */
	BCM2837_SPI_CLOCK_DIVIDER_16    = 16    /*!< 16 = 64ns = 15.625MHz */
	BCM2837_SPI_CLOCK_DIVIDER_8     = 8     /*!< 8 = 32ns = 31.25MHz */
	BCM2837_SPI_CLOCK_DIVIDER_4     = 4     /*!< 4 = 16ns = 62.5MHz */
	BCM2837_SPI_CLOCK_DIVIDER_2     = 2     /*!< 2 = 8ns = 125MHz, fastest you can get */
	BCM2837_SPI_CLOCK_DIVIDER_1     = 1     /*!< 1 = 262.144us = 3.814697260kHz, same as 0/65536 */

	//I2C define
	BCM2837_BSC_C    = 0x0000 /*!< BSC Master Control */
	BCM2837_BSC_S    = 0x0004 /*!< BSC Master Status */
	BCM2837_BSC_DLEN = 0x0008 /*!< BSC Master Data Length */
	BCM2837_BSC_A    = 0x000c /*!< BSC Master Slave Address */
	BCM2837_BSC_FIFO = 0x0010 /*!< BSC Master Data FIFO */
	BCM2837_BSC_DIV  = 0x0014 /*!< BSC Master Clock Divider */
	BCM2837_BSC_DEL  = 0x0018 /*!< BSC Master Data Delay */
	BCM2837_BSC_CLKT = 0x001c /*!< BSC Master Clock Stretch Timeout */

	/* Register masks for BSC_C */
	BCM2837_BSC_C_I2CEN   = 0x00008000 /*!< I2C Enable, 0 = disabled, 1 = enabled */
	BCM2837_BSC_C_INTR    = 0x00000400 /*!< Interrupt on RX */
	BCM2837_BSC_C_INTT    = 0x00000200 /*!< Interrupt on TX */
	BCM2837_BSC_C_INTD    = 0x00000100 /*!< Interrupt on DONE */
	BCM2837_BSC_C_ST      = 0x00000080 /*!< Start transfer, 1 = Start a new transfer */
	BCM2837_BSC_C_CLEAR_1 = 0x00000020 /*!< Clear FIFO Clear */
	BCM2837_BSC_C_CLEAR_2 = 0x00000010 /*!< Clear FIFO Clear */
	BCM2837_BSC_C_READ    = 0x00000001 /*!<	Read transfer */

	/* Register masks for BSC_S */
	BCM2837_BSC_S_CLKT = 0x00000200 /*!< Clock stretch timeout */
	BCM2837_BSC_S_ERR  = 0x00000100 /*!< ACK error */
	BCM2837_BSC_S_RXF  = 0x00000080 /*!< RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
	BCM2837_BSC_S_TXE  = 0x00000040 /*!< TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
	BCM2837_BSC_S_RXD  = 0x00000020 /*!< RXD FIFO contains data */
	BCM2837_BSC_S_TXD  = 0x00000010 /*!< TXD FIFO can accept data */
	BCM2837_BSC_S_RXR  = 0x00000008 /*!< RXR FIFO needs reading (full) */
	BCM2837_BSC_S_TXW  = 0x00000004 /*!< TXW FIFO needs writing (full) */
	BCM2837_BSC_S_DONE = 0x00000002 /*!< Transfer DONE */
	BCM2837_BSC_S_TA   = 0x00000001 /*!< Transfer Active */

	BCM2837_BSC_FIFO_SIZE = 16 /*!< BSC FIFO size */

	// i2c clock div
	BCM2837_I2C_CLOCK_DIVIDER_2500 = 2500 /*!< 2500 = 10us = 100 kHz */
	BCM2837_I2C_CLOCK_DIVIDER_626  = 626  /*!< 622 = 2.504us = 399.3610 kHz */
	BCM2837_I2C_CLOCK_DIVIDER_150  = 150  /*!< 150 = 60ns = 1.666 MHz (default at reset) */
	BCM2837_I2C_CLOCK_DIVIDER_148  = 148  /*!< 148 = 59ns = 1.689 MHz */

	//I2C reasoncode
	BCM2837_I2C_REASON_OK         uint32 = 0x00 /*!< Success */
	BCM2837_I2C_REASON_ERROR_NACK uint32 = 0x01 /*!< Received a NACK */
	BCM2837_I2C_REASON_ERROR_CLKT uint32 = 0x02 /*!< Received Clock Stretch Timeout */
	BCM2837_I2C_REASON_ERROR_DATA uint32 = 0x04 /*!< Not all data is sent / received */

	//define for system timer
	BCM2837_ST_CS  uint32 = 0x0000 /*!< System Timer Control/Status */
	BCM2837_ST_CLO uint32 = 0x0004 /*!< System Timer Counter Lower 32 bits */
	BCM2837_ST_CHI uint32 = 0x0008 /*!< System Timer Counter Upper 32 bits */

	//define for pwm
	BCM2837_PWM_CONTROL uint32 = (0 * 4)
	BCM2837_PWM_STATUS  uint32 = (1 * 4)
	BCM2837_PWM_DMAC    uint32 = (2 * 4)
	BCM2837_PWM0_RANGE  uint32 = (4 * 4)
	BCM2837_PWM0_DATA   uint32 = (5 * 4)
	BCM2837_PWM_FIF1    uint32 = (6 * 4)
	BCM2837_PWM1_RANGE  uint32 = (8 * 4)
	BCM2837_PWM1_DATA   uint32 = (9 * 4)

	/* Defines for PWM Clock, word offsets (ie 4 byte multiples) */
	BCM2837_PWMCLK_CNTL uint32 = (40 * 4)
	BCM2837_PWMCLK_DIV  uint32 = (41 * 4)
	BCM2837_PWM_PASSWRD uint32 = (0x5A << 24) /*!< Password to enable setting PWM clock */

	BCM2837_PWM1_MS_MODE  uint32 = 0x8000 /*!< Run in Mark/Space mode */
	BCM2837_PWM1_USEFIFO  uint32 = 0x2000 /*!< Data from FIFO */
	BCM2837_PWM1_REVPOLAR uint32 = 0x1000 /*!< Reverse polarity */
	BCM2837_PWM1_OFFSTATE uint32 = 0x0800 /*!< Ouput Off state */
	BCM2837_PWM1_REPEATFF uint32 = 0x0400 /*!< Repeat last value if FIFO empty */
	BCM2837_PWM1_SERIAL   uint32 = 0x0200 /*!< Run in serial mode */
	BCM2837_PWM1_ENABLE   uint32 = 0x0100 /*!< Channel Enable */

	BCM2837_PWM0_MS_MODE   uint32 = 0x0080 /*!< Run in Mark/Space mode */
	BCM2837_PWM_CLEAR_FIFO uint32 = 0x0040 /*!< Clear FIFO */
	BCM2837_PWM0_USEFIFO   uint32 = 0x0020 /*!< Data from FIFO */
	BCM2837_PWM0_REVPOLAR  uint32 = 0x0010 /*!< Reverse polarity */
	BCM2837_PWM0_OFFSTATE  uint32 = 0x0008 /*!< Ouput Off state */
	BCM2837_PWM0_REPEATFF  uint32 = 0x0004 /*!< Repeat last value if FIFO empty */
	BCM2837_PWM0_SERIAL    uint32 = 0x0002 /*!< Run in serial mode */
	BCM2837_PWM0_ENABLE    uint32 = 0x0001 /*!< Channel Enable */

	//define for pwm clock div
	BCM2837_PWM_CLOCK_DIVIDER_2048 uint32 = 2048 /*!< 2048 = 9.375kHz */
	BCM2837_PWM_CLOCK_DIVIDER_1024 uint32 = 1024 /*!< 1024 = 18.75kHz */
	BCM2837_PWM_CLOCK_DIVIDER_512  uint32 = 512  /*!< 512 = 37.5kHz */
	BCM2837_PWM_CLOCK_DIVIDER_256  uint32 = 256  /*!< 256 = 75kHz */
	BCM2837_PWM_CLOCK_DIVIDER_128  uint32 = 128  /*!< 128 = 150kHz */
	BCM2837_PWM_CLOCK_DIVIDER_64   uint32 = 64   /*!< 64 = 300kHz */
	BCM2837_PWM_CLOCK_DIVIDER_32   uint32 = 32   /*!< 32 = 600.0kHz */
	BCM2837_PWM_CLOCK_DIVIDER_16   uint32 = 16   /*!< 16 = 1.2MHz */
	BCM2837_PWM_CLOCK_DIVIDER_8    uint32 = 8    /*!< 8 = 2.4MHz */
	BCM2837_PWM_CLOCK_DIVIDER_4    uint32 = 4    /*!< 4 = 4.8MHz */
	BCM2837_PWM_CLOCK_DIVIDER_2    uint32 = 2    /*!< 2 = 9.6MHz, fastest you can get */
	BCM2837_PWM_CLOCK_DIVIDER_1    uint32 = 1    /*!< 1 = 4.6875kHz, same as divider*/
)

var Bcm2837_peripherals_base uint32
var Bcm2837_peripherals_size uint32

// define the mmu address
var Bcm2837_gpio uint32
var Bcm2837_pwm uint32
var Bcm2837_clk uint32
var Bcm2837_pads uint32
var Bcm2837_spi0 uint32
var Bcm2837_bsc0 uint32
var Bcm2837_bsc1 uint32
var Bcm2837_st uint32

var Mapp []byte

func Bcm2837_init() int {
	// find the io peripheral base and range
	f, err := os.OpenFile(BCM2837_PRI3B_DT_FILENAME, os.O_RDONLY, 0)

	if err != nil {
		fmt.Println("open range file err")
	}
	defer f.Close()

	var buf []byte = make([]byte, 4)
	f.ReadAt(buf, BCM2837_PRI3_DT_PERI_BASE_ADDRESS_OFFSET)
	bytesBuffer := bytes.NewBuffer(buf)
	binary.Read(bytesBuffer, binary.BigEndian, &Bcm2837_peripherals_base)

	f.ReadAt(buf, BCM2837_PRI3_DT_PERI_SIZE_OFFSET)
	bytesBuffer = bytes.NewBuffer(buf)
	binary.Read(bytesBuffer, binary.BigEndian, &Bcm2837_peripherals_size)

	//fmt.Printf("get peripherals base:%x size:%x\n" , Bcm2837_peripherals_base , Bcm2837_peripherals_size )
	if os.Geteuid() == 0 {

		/* open the master /dev/mem device */
		f, err := os.OpenFile("/dev/mem", os.O_RDWR, 0)
		if err != nil {
			fmt.Println("Open mem error")
		}

		Mapp, err = syscall.Mmap(int(f.Fd()), int64(Bcm2837_peripherals_base), int(Bcm2837_peripherals_size), syscall.PROT_READ|syscall.PROT_WRITE, syscall.MAP_SHARED)

		if err != nil {
			fmt.Println("mmap error")
		}

		//strat find the gpio register
		Bcm2837_gpio = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_GPIO_BASE)
		Bcm2837_pwm = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_GPIO_PWM)
		Bcm2837_clk = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_CLOCK_BASE)
		Bcm2837_pads = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_GPIO_PADS)
		Bcm2837_spi0 = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_SPI0_BASE)
		Bcm2837_bsc0 = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_BSC0_BASE)
		Bcm2837_bsc1 = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_BSC1_BASE)
		Bcm2837_st = *(*uint32)(unsafe.Pointer(&Mapp)) + uint32(BCM2837_ST_BASE)

		return 0
	} else {
		fmt.Println("please use root execute")
		panic(err)
		return -1
	}
}

func Bcm2837_close() int {
	syscall.Munmap(Mapp)

	Bcm2837_gpio = 0
	Bcm2837_pwm = 0
	Bcm2837_clk = 0
	Bcm2837_pads = 0
	Bcm2837_spi0 = 0
	Bcm2837_bsc0 = 0
	Bcm2837_bsc1 = 0
	Bcm2837_st = 0

	return 0
}

/* PWM */
func Bcm2837_pwm_set_clock(div uint32) {
	if (Bcm2837_clk == 0) || (Bcm2837_pwm == 0) {
		panic(err)
	}
	/* From Gerts code */
	div &= 0xfff

	/* Stop PWM clock */
	Bcm2837_peri_write(Bcm2837_clk+BCM2837_PWMCLK_CNTL, BCM2837_PWM_PASSWRD|0x01)
	time.Sleep(time.Microsecond * 100)
	/* Wait for the clock to be not busy */
	for {
		if (Bcm2837_peri_read(Bcm2837_clk+BCM2837_PWMCLK_CNTL) & 0x80) == 0 {
			break
		}

	}
	time.Sleep(time.Millisecond)
	/* set the clock divider and enable PWM clock */
	Bcm2837_peri_write(Bcm2837_clk+BCM2837_PWMCLK_DIV, BCM2837_PWM_PASSWRD|(div<<12))
	Bcm2837_peri_write(Bcm2837_clk+BCM2837_PWMCLK_CNTL, BCM2837_PWM_PASSWRD|0x11) /* Source=osc and enable */
}

func Bcm2837_pwm_set_mode(channel, mark, enable uint8) {
	if (Bcm2837_clk == 0) || (Bcm2837_pwm == 0) {
		panic(err)
		return
	}

	var ctrl uint32 = Bcm2837_peri_read(Bcm2837_pwm + BCM2837_PWM_CONTROL)
	if channel == 0 {
		if mark == 1 {
			ctrl |= BCM2837_PWM0_MS_MODE
		} else {
			ctrl &= ^BCM2837_PWM0_MS_MODE
		}
		if enable == 1 {
			ctrl |= BCM2837_PWM0_ENABLE
		} else {
			ctrl &= ^BCM2837_PWM0_ENABLE
		}
	} else if channel == 1 {
		if mark == 1 {
			ctrl |= BCM2837_PWM1_MS_MODE
		} else {
			ctrl &= ^BCM2837_PWM1_MS_MODE
		}
		if enable == 1 {
			ctrl |= BCM2837_PWM1_ENABLE
		} else {
			ctrl &= ^BCM2837_PWM1_ENABLE
		}
	}

	/* If you use the barrier here, wierd things happen, and the commands dont work */
	Bcm2837_peri_write(Bcm2837_pwm+BCM2837_PWM_CONTROL, ctrl) //origin   Bcm2837_peri_write_nb
}

func Bcm2837_pwm_set_range(channel uint8, rg uint32) {

	if (Bcm2837_clk == 0) || (Bcm2837_pwm == 0) {
		panic(err)
		return /* bcm2835_init() failed or not root */
	}

	if channel == 0 {
		Bcm2837_peri_write(Bcm2837_pwm+BCM2837_PWM0_RANGE, rg)
	} else if channel == 1 {
		Bcm2837_peri_write(Bcm2837_pwm+BCM2837_PWM1_RANGE, rg)
	}

}

func Bcm2837_pwm_set_data(channel uint8, data uint32) {

	if (Bcm2837_clk == 0) || (Bcm2837_pwm == 0) {
		panic(err)
		return /* bcm2835_init() failed or not root */
	}

	if channel == 0 {
		Bcm2837_peri_write(Bcm2837_pwm+BCM2837_PWM0_DATA, data)
	} else if channel == 1 {
		Bcm2837_peri_write(Bcm2837_pwm+BCM2837_PWM1_DATA, data)
	}
}

var i2c_byte_wait_us uint32

func Bcm2837_i2c_begin() uint32 {
	var div uint32

	if (Bcm2837_bsc0 == 0) || (Bcm2837_bsc1 == 0) {
		panic(err)
		return -1
	}

	var paddr uint32 = Bcm2837_bsc1 + BCM2837_BSC_DIV
	/* Set the I2C/BSC1 pins to the Alt 0 function to enable I2C access on them */
	Bcm2837_gpio_fsel(RPI_3B_GPIO_J8_03, BCM2837_GPIO_FSEL_ALT0) /* SDA */
	Bcm2837_gpio_fsel(RPI_3B_GPIO_J8_05, BCM2837_GPIO_FSEL_ALT0) /* SCL */

	/* Read the clock divider register */
	div = Bcm2837_peri_read(paddr)

	/* Calculate time for transmitting one byte
	// 1000000 = micros seconds in a second
	// 9 = Clocks per byte : 8 bits + ACK
	*/
	i2c_byte_wait_us = uint32((float32(div) / BCM2837_CORE_CLK_HZ) * 1000000 * 9) //TODO this may wrong

	return 0
}

func Bcm2837_i2c_end() {
	/* Set all the I2C/BSC1 pins back to input */
	Bcm2837_gpio_fsel(RPI_3B_GPIO_J8_03, BCM2837_GPIO_FSEL_INPT) /* SDA */
	Bcm2837_gpio_fsel(RPI_3B_GPIO_J8_05, BCM2837_GPIO_FSEL_INPT) /* SCL */
}

func Bcm2837_i2c_setSlaveAddress(addr uint8) {
	/* Set I2C Device Address */
	var paddr uint32 = Bcm2837_bsc1 + BCM2837_BSC_A

	Bcm2837_peri_write(paddr, uint32(addr))
}

func Bcm2837_i2c_setClockDivider(div uint32) {
	var paddr uint32 = Bcm2837_bsc1 + BCM2837_BSC_DIV

	Bcm2837_peri_write(paddr, div)
	/* Calculate time for transmitting one byte
	   // 1000000 = micros seconds in a second
	   // 9 = Clocks per byte : 8 bits + ACK
	*/
	i2c_byte_wait_us = uint32((float32(div) / BCM2837_CORE_CLK_HZ) * 1000000 * 9)
}

/* set I2C clock divider by means of a baudrate number */
func Bcm2837_i2c_set_baudrate(baudrate uint32) {
	var divider uint32
	/* use 0xFFFE mask to limit a max value and round down any odd number */
	divider = (BCM2837_CORE_CLK_HZ / baudrate) & 0xFFFE
	Bcm2837_i2c_setClockDivider(uint32(divider))
}

//TODO 2017.09.01 ready for i2c write
func Bcm2837_i2c_write(buf []byte, len uint32) uint32 {
	/* i2c 1*/
	var dlen uint32 = Bcm2837_bsc1 + BCM2837_BSC_DLEN
	var fifo uint32 = Bcm2837_bsc1 + BCM2837_BSC_FIFO
	var stat uint32 = Bcm2837_bsc1 + BCM2837_BSC_S
	var ctrl uint32 = Bcm2837_bsc1 + BCM2837_BSC_C

	var remain uint32 = len
	var iCnt uint32 = 0
	var reason uint32 = BCM2837_I2C_REASON_OK

	/* Clear FIFO */
	Bcm2837_peri_set_bits(ctrl, BCM2837_BSC_C_CLEAR_1, BCM2837_BSC_C_CLEAR_1)
	/* Clear Status */
	Bcm2837_peri_write(stat, BCM2837_BSC_S_CLKT|BCM2837_BSC_S_ERR|BCM2837_BSC_S_DONE)
	/* Set Data Length */
	Bcm2837_peri_write(dlen, len)
	/* pre populate FIFO with max buffer */
	for false != (0 != remain) && (iCnt < BCM2837_BSC_FIFO_SIZE) {
		Bcm2837_peri_write(fifo, uint32(buf[iCnt]))
		iCnt++
		remain--
	}

	/* Enable device and start transfer */
	Bcm2837_peri_write(ctrl, BCM2837_BSC_C_I2CEN|BCM2837_BSC_C_ST)

	/* Transfer is over when BCM2835_BSC_S_DONE */
	for 0 == (Bcm2837_peri_read(stat) & BCM2837_BSC_S_DONE) {
		for false != ((0 != remain) && (0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_TXD))) {
			/* Write to FIFO */
			Bcm2837_peri_write(fifo, uint32(buf[iCnt]))
			iCnt++
			remain--
		}
	}

	/* Received a NACK */
	if 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_ERR) {
		reason = BCM2837_I2C_REASON_ERROR_NACK
	} else if 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_CLKT) {
		/* Received Clock Stretch Timeout */
		reason = BCM2837_I2C_REASON_ERROR_CLKT
	} else if 0 != remain {
		/* Not all data is sent */
		reason = BCM2837_I2C_REASON_ERROR_DATA
	}

	Bcm2837_peri_set_bits(ctrl, BCM2837_BSC_S_DONE, BCM2837_BSC_S_DONE)

	return reason
}

/* Read an number of bytes from I2C */
func Bcm2837_i2c_read(buf []byte, len uint32) byte {

	var dlen uint32 = Bcm2837_bsc1 + BCM2837_BSC_DLEN
	var fifo uint32 = Bcm2837_bsc1 + BCM2837_BSC_FIFO
	var stat uint32 = Bcm2837_bsc1 + BCM2837_BSC_S
	var ctrl uint32 = Bcm2837_bsc1 + BCM2837_BSC_C

	var remain uint32 = len
	var iCnt uint32 = 0
	var reason uint32 = BCM2837_I2C_REASON_OK

	/* Clear FIFO */
	Bcm2837_peri_set_bits(ctrl, BCM2837_BSC_C_CLEAR_1, BCM2837_BSC_C_CLEAR_1)
	/* Clear Status */
	Bcm2837_peri_write(stat, BCM2837_BSC_S_CLKT|BCM2837_BSC_S_ERR|BCM2837_BSC_S_DONE)
	/* Set Data Length */
	Bcm2837_peri_write(dlen, len)
	/* Start read */
	Bcm2837_peri_write(ctrl, BCM2837_BSC_C_I2CEN|BCM2837_BSC_C_ST|BCM2837_BSC_C_READ)

	/* wait for transfer to complete */
	for 0 == (Bcm2837_peri_read(stat) & BCM2837_BSC_S_DONE) {
		/* we must empty the FIFO as it is populated and not use any delay */
		for 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_RXD) {
			/* Read from FIFO, no barrier */
			buf[iCnt] = byte(Bcm2837_peri_read(fifo))
			iCnt++
			remain--
		}
	}

	/* transfer has finished - grab any remaining stuff in FIFO */
	for false != ((0 != remain) && (0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_RXD))) {
		/* Read from FIFO, no barrier */
		buf[iCnt] = byte(Bcm2837_peri_read(fifo))
		iCnt++
		remain--
	}

	/* Received a NACK */
	if 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_ERR) {
		reason = BCM2837_I2C_REASON_ERROR_NACK
	} else if 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_CLKT) {
		/* Received Clock Stretch Timeout */
		reason = BCM2837_I2C_REASON_ERROR_CLKT
	} else if 0 != remain {
		/* Not all data is received */
		reason = BCM2837_I2C_REASON_ERROR_DATA
	}

	Bcm2837_peri_set_bits(ctrl, BCM2837_BSC_S_DONE, BCM2837_BSC_S_DONE)

	return byte(reason)
}

/* Read an number of bytes from I2C sending a repeated start after writing
// the required register. Only works if your device supports this mode
*/
func Bcm2837_i2c_read_register_rs(regaddr, buf []byte, len uint32) uint8 {

	var dlen uint32 = Bcm2837_bsc1 + BCM2837_BSC_DLEN
	var fifo uint32 = Bcm2837_bsc1 + BCM2837_BSC_FIFO
	var stat uint32 = Bcm2837_bsc1 + BCM2837_BSC_S
	var ctrl uint32 = Bcm2837_bsc1 + BCM2837_BSC_C

	var remain uint32 = len
	var iCnt uint32 = 0
	var reason uint32 = BCM2837_I2C_REASON_OK

	/* Clear FIFO */
	Bcm2837_peri_set_bits(ctrl, BCM2837_BSC_C_CLEAR_1, BCM2837_BSC_C_CLEAR_1)
	/* Clear Status */
	Bcm2837_peri_write(stat, BCM2837_BSC_S_CLKT|BCM2837_BSC_S_ERR|BCM2837_BSC_S_DONE)
	/* Set Data Length */
	Bcm2837_peri_write(dlen, 1)
	/* Enable device and start transfer */
	Bcm2837_peri_write(ctrl, BCM2837_BSC_C_I2CEN)
	Bcm2837_peri_write(fifo, uint32(regaddr[0]))
	Bcm2837_peri_write(ctrl, BCM2837_BSC_C_I2CEN|BCM2837_BSC_C_ST)

	/* poll for transfer has started */
	for 0 == (Bcm2837_peri_read(stat) & BCM2837_BSC_S_TA) {
		if 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_DONE) {
			break
		}
	}

	/* Send a repeated start with read bit set in address */
	Bcm2837_peri_write(dlen, len)
	Bcm2837_peri_write(ctrl, BCM2837_BSC_C_I2CEN|BCM2837_BSC_C_ST|BCM2837_BSC_C_READ)

	/* Wait for write to complete and first byte back. */

	time.Sleep(time.Microsecond * 6) //TODO this time maybe wrong
	//Bcm2837_delayMicroseconds(i2c_byte_wait_us * 3)

	/* wait for transfer to complete */
	for 0 == (Bcm2837_peri_read(stat) & BCM2837_BSC_S_DONE) {
		for false != ((0 != remain) && bool(Bcm2837_peri_read(stat)&BCM2837_BSC_S_RXD)) {
			buf[iCnt] = byte(Bcm2837_peri_read(fifo))
			iCnt++
			remain--
		}
	}

	for 0 == (Bcm2837_peri_read(stat) & BCM2837_BSC_S_DONE) {

		/* we must empty the FIFO as it is populated and not use any delay */
		for false != ((0 != remain) && bool(Bcm2837_peri_read(stat)&BCM2837_BSC_S_RXD)) {
			/* Read from FIFO */
			buf[iCnt] = byte(Bcm2837_peri_read(fifo))
			iCnt++
			remain--
		}
	}

	/* transfer has finished - grab any remaining stuff in FIFO */
	for false != ((0 != remain) && bool(Bcm2837_peri_read(stat)&BCM2837_BSC_S_RXD)) {
		/* Read from FIFO */
		buf[iCnt] = byte(Bcm2837_peri_read(fifo))
		iCnt++
		remain--
	}

	/* Received a NACK */
	if 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_ERR) {
		reason = BCM2837_I2C_REASON_ERROR_NACK
	} else if 0 != (Bcm2837_peri_read(stat) & BCM2837_BSC_S_CLKT) {
		/* Received Clock Stretch Timeout */
		reason = BCM2837_I2C_REASON_ERROR_CLKT
	} else if 0 != (remain) {
		/* Not all data is sent */
		reason = BCM2837_I2C_REASON_ERROR_DATA
	}

	Bcm2837_peri_set_bits(ctrl, BCM2837_BSC_S_DONE, BCM2837_BSC_S_DONE)

	return uint8(reason)
}

/*
 *  read with memory form peripheral
 */
func Bcm2837_peri_read(paddr uint32) uint32 {
	var ret uint32
	var p = uintptr(paddr)
	ret = *(*uint32)(unsafe.Pointer(p))
	return ret
}

/*
 *  write with memory to peripheral
 */
func Bcm2837_peri_write(paddr, value uint32) {
	var p = uintptr(paddr)
	*(*uint32)(unsafe.Pointer(p)) = value
}

/* Set/clear only the bits in value covered by the mask
 * This is not atomic - can be interrupted.
 */
func Bcm2837_peri_set_bits(paddr, value, mask uint32) {

	var v uint32 = Bcm2837_peri_read(paddr)
	v = (v & ^mask) | (value & mask) //TODO maybe wrong
	fmt.Printf("write address:0x%x val:0x%x\n", paddr, v)
	Bcm2837_peri_write(paddr, v)
}

/*
// Low level convenience functions
*/

/* Function select
// pin is a BCM2835 GPIO pin number NOT RPi pin number
//      There are 6 control registers, each control the function0s of a block
//      of 10 pins.
//      Each control register has 10 sets of 3 bits per GPIO pin:
//
//      000 = GPIO Pin X is an input
//      001 = GPIO Pin X is an output
//      100 = GPIO Pin X takes alternate function 0
//      101 = GPIO Pin X takes alternate function 1
//      110 = GPIO Pin X takes alternate function 2
//      111 = GPIO Pin X takes alternate function 3
//      011 = GPIO Pin X takes alternate function 4
//      010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//      X / 10 + ((X % 10) * 3)
*/

func Bcm2837_gpio_fsel(pin, mode uint8) {
	var paddr uint32 = uint32(Bcm2837_gpio + BCM2837_GPFSEL0 + uint32((pin/10)*4))
	var shift uint8 = 3 * (pin % 10)
	var mask uint32 = BCM2837_GPIO_FSEL_MASK << shift
	var val uint32 = (uint32(mode)) << shift
	fmt.Printf("address: 0x%x val: 0x%x \n", paddr, val)
	Bcm2837_peri_set_bits(paddr, val, mask)
}

/* Set output pin */
func Bcm2837_gpio_set(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPSET0 + uint32(pin/32)
	var shift uint8 = pin % 32
	Bcm2837_peri_write(paddr, 1<<shift)
}

/* Clear output pin */
func Bcm2837_gpio_clr(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPCLR0 + uint32(pin/32)
	var shift uint8 = pin % 32
	Bcm2837_peri_write(paddr, 1<<shift)
}

/* Set all output pins in the mask */
func Bcm2837_gpio_set_multi(mask uint32) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPSET0
	Bcm2837_peri_write(paddr, mask)
}

/* clear all output pins in the mask */
func Bcm2837_gpio_clr_multi(mask uint32) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPCLR0
	Bcm2837_peri_write(paddr, mask)
}

/* read input pin */
func Bcm2837_gpio_lev(pin uint8) uint8 {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPLEV0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var val uint32 = Bcm2837_peri_read(paddr)

	if (val & (1 << shift)) != 0 {
		return HIGH
	} else {
		return LOW
	}
}

/* See if an event detection bit is set
// Sigh can't support interrupts yet
*/
func Bcm2837_gpio_eds(pin uint8) uint8 {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPEDS0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = Bcm2837_peri_read(paddr)

	if (value & (1 << shift)) != 0 {
		return HIGH
	} else {
		return LOW
	}
}

func Bcm2837_gpio_eds_multi(mask uint32) uint32 {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPEDS0
	var value uint32 = Bcm2837_peri_read(paddr)
	return (value & mask)
}

/* Write a 1 to clear the bit in EDS */
func Bcm2837_gpio_set_eds(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPEDS0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_write(paddr, value)
}

func Bcm2837_gpio_set_eds_multi(mask uint32) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPEDS0
	Bcm2837_peri_write(paddr, mask)
}

/* Rising edge detect enable */
func Bcm2837_gpio_ren(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPREN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, value, value)
}
func Bcm2837_gpio_clr_ren(pin uint8) {

	var paddr uint32 = Bcm2837_gpio + BCM2837_GPREN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, 0, value)
}

/* Falling edge detect enable */
func Bcm2837_gpio_fen(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPFEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, value, value)
}

func Bcm2837_gpio_clr_fen(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPFEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, 0, value)
}

/* High detect enable */
func Bcm2837_gpio_hen(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPHEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, value, value)
}

func Bcm2837_gpio_clr_hen(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPHEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, 0, value)
}

/* Low detect enable */
func Bcm2837_gpio_len(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPLEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, value, value)
}

func Bcm2837_gpio_clr_len(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPLEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, 0, value)
}

/* Async rising edge detect enable */
func Bcm2837_gpio_aren(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPAREN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, value, value)
}
func Bcm2837_gpio_clr_aren(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPAREN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, 0, value)
}

/* Async falling edge detect enable */
func Bcm2837_gpio_afen(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPAFEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, value, value)
}
func Bcm2837_gpio_clr_afen(pin uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPAFEN0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 1 << shift
	Bcm2837_peri_set_bits(paddr, 0, value)
}

/* Read GPIO pad behaviour for groups of GPIOs */
func Bcm2837_gpio_pad(group uint8) uint32 {
	if Bcm2837_pads == 0 {
		panic(err)
		return 0
	}

	var paddr uint32 = Bcm2837_pads + BCM2837_PADS_GPIO_0_27 + uint32(group)
	return Bcm2837_peri_read(paddr)
}

/* Set GPIO pad behaviour for groups of GPIOs
// powerup value for all pads is
// BCM2835_PAD_SLEW_RATE_UNLIMITED | BCM2835_PAD_HYSTERESIS_ENABLED | BCM2835_PAD_DRIVE_8mA
*/
func Bcm2837_gpio_set_pad(group uint8, control uint32) {
	if Bcm2837_pads == 0 {
		panic(err)
		return
	}
	var paddr uint32 = Bcm2837_pads + BCM2837_PADS_GPIO_0_27 + uint32(group)
	Bcm2837_peri_write(paddr, control|BCM2837_PAD_PASSWRD)
}

/* Set pullup/down */
func Bcm2837_gpio_pud(pud uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPPUD
	Bcm2837_peri_write(paddr, uint32(pud))
}

/* Pullup/down clock
// Clocks the value of pud into the GPIO pin
*/
func Bcm2837_gpio_pudclk(pin, on uint8) {
	var paddr uint32 = Bcm2837_gpio + BCM2837_GPPUDCLK0 + uint32(pin/32)
	var shift uint8 = pin % 32
	var value uint32 = 0
	if on != 0 {
		value = 1
	}
	Bcm2837_peri_write(paddr, value<<shift)
}

/* Set the pullup/down resistor for a pin
//
// The GPIO Pull-up/down Clock Registers control the actuation of internal pull-downs on
// the respective GPIO pins. These registers must be used in conjunction with the GPPUD
// register to effect GPIO Pull-up/down changes. The following sequence of events is
// required:
// 1. Write to GPPUD to set the required control signal (i.e. Pull-up or Pull-Down or neither
// to remove the current Pull-up/down)
// 2. Wait 150 cycles ? this provides the required set-up time for the control signal
// 3. Write to GPPUDCLK0/1 to clock the control signal into the GPIO pads you wish to
// modify ? NOTE only the pads which receive a clock will be modified, all others will
// retain their previous state.
// 4. Wait 150 cycles ? this provides the required hold time for the control signal
// 5. Write to GPPUD to remove the control signal
// 6. Write to GPPUDCLK0/1 to remove the clock
//
// RPi has P1-03 and P1-05 with 1k8 pullup resistor
*/
func Bcm2837_gpio_set_pud(pin, pud uint8) {
	Bcm2837_gpio_pud(pud)

	time.Sleep(time.Microsecond * 10)
	Bcm2837_gpio_pudclk(pin, 1)
	time.Sleep(time.Microsecond * 10)

	Bcm2837_gpio_pud(BCM2837_GPIO_PUD_OFF)
	Bcm2837_gpio_pudclk(pin, 0)
}

func main() {
	fmt.Println("starting...")
	if 0 != Bcm2837_init() {
		Bcm2837_close()
		panic(err)
		return
	}

	//test for pwm
	//Bcm2837_gpio_fsel(RPI_3B_GPIO_J8_12,uint8(BCM2837_GPIO_FSEL_ALT5))
	//
	//Bcm2837_pwm_set_clock(BCM2837_PWM_CLOCK_DIVIDER_16)
	//
	//Bcm2837_pwm_set_mode(0, 1, 1 )
	//
	//Bcm2837_pwm_set_range(0,1024 )
	//
	//Bcm2837_pwm_set_data(0,10 )
	//
	//var data uint32 = 1
	//for{
	//	for {
	//		time.Sleep(time.Millisecond*2)
	//		data ++
	//		if data > 768{
	//			break
	//		}
	//		Bcm2837_pwm_set_data(0,data )
	//	}
	//	for {
	//		time.Sleep(time.Millisecond*2)
	//		data --
	//		if data < 2 {
	//			data = 1
	//			break
	//		}
	//		Bcm2837_pwm_set_data(0,data )
	//	}
	//
	//}

	//test for gpio
	//Bcm2837_gpio_fsel(RPI_3B_GPIO_J8_12,uint8(BCM2837_GPIO_FSEL_OUTP))
	//Bcm2837_gpio_set_pud(RPI_3B_GPIO_J8_12, BCM2837_GPIO_PUD_UP)
	//Bcm2837_gpio_set(RPI_3B_GPIO_J8_12)
	//time.Sleep(time.Second * 3)
	//Bcm2837_gpio_clr(RPI_3B_GPIO_J8_12)

	//test for i2c

	err := Bcm2837_i2c_begin()
	if err != 0 {
		fmt.Println("Bcm2837_i2c_begin failed")
	}
	var slave_addr uint8 = 0x37
	var clk_div uint32 = BCM2837_I2C_CLOCK_DIVIDER_148
	Bcm2837_i2c_setSlaveAddress(slave_addr)

	Bcm2837_i2c_setClockDivider(clk_div)

	//try read

}
