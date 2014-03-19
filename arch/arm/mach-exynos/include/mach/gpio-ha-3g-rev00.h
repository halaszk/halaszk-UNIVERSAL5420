/*
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __MACH_GPIO_EXYNOS5420_UNIVERSAL_EVT0_H
#define __MACH_GPIO_EXYNOS5420_UNIVERSAL_EVT0_H __FILE__

#define	GPIO_BT_UART_RXD		EXYNOS5420_GPA0(0)
#define	GPIO_BT_UART_TXD		EXYNOS5420_GPA0(1)
#define	GPIO_BT_UART_CTS		EXYNOS5420_GPA0(2)
#define	GPIO_BT_UART_RTS		EXYNOS5420_GPA0(3)

#define	GPIO_GPS_RXD		    EXYNOS5420_GPA0(4)
#define	GPIO_GPS_RXD_AF         2
#define	GPIO_GPS_TXD		    EXYNOS5420_GPA0(5)
#define	GPIO_GPS_TXD_AF         2
#define	GPIO_GPS_CTS		    EXYNOS5420_GPA0(6)
#define	GPIO_GPS_CTS_AF         2
#define	GPIO_GPS_RTS		    EXYNOS5420_GPA0(7)
#define	GPIO_GPS_RTS_AF         2

#define	GPIO_AP_RXD				EXYNOS5420_GPA1(0)
#define	GPIO_AP_TXD				EXYNOS5420_GPA1(1)
#define	GPIO_NFC_SDA_18V		EXYNOS5420_GPA1(2)
#define	GPIO_NFC_SCL_18V			EXYNOS5420_GPA1(3)
#define	GPIO_BARCODE_SCL_1_8V			EXYNOS5420_GPA1(4)
#define	GPIO_BARCODE_SDA_1_8V			EXYNOS5420_GPA1(5)

#define	GPIO_SHUB_SPI_SCK		EXYNOS5420_GPA2(0)
#define	GPIO_SHUB_SPI_SSN		EXYNOS5420_GPA2(1)
#define	GPIO_SHUB_SPI_MISO		EXYNOS5420_GPA2(2)
#define	GPIO_SHUB_SPI_MOSI		EXYNOS5420_GPA2(3)

#define	GPIO_BTP_SPI_CLK			EXYNOS5420_GPA2(4)
#define	GPIO_BTP_SPI_CS_N		EXYNOS5420_GPA2(5)
#define	GPIO_BTP_SPI_MISO		EXYNOS5420_GPA2(6)
#define	GPIO_BTP_SPI_MOSI		EXYNOS5420_GPA2(7)

#define	GPIO_FUEL_SDA_18V		EXYNOS5420_GPB0(3)
#define	GPIO_FUEL_SCL_18V		EXYNOS5420_GPB0(4)

#define	GPIO_FPGA_SPI_SI			EXYNOS5420_GPB1(0)
#define	GPIO_YMU_SPI_SCK		EXYNOS5420_GPB1(1)
#define	GPIO_YMU_SPI_SS_N		EXYNOS5420_GPB1(2)
#define	GPIO_YMU_SPI_MISO		EXYNOS5420_GPB1(3)
#define	GPIO_YMU_SPI_MOSI		EXYNOS5420_GPB1(4)

#define	GPIO_VIBTONE_PWM		EXYNOS5420_GPB2(0)
#define	GPIO_COVER_ID			EXYNOS5420_GPB2(1)
#define	GPIO_AP_PMIC_SDA		EXYNOS5420_GPB2(2)
#define	GPIO_AP_PMIC_SCL			EXYNOS5420_GPB2(3)

#define	GPIO_TSP_SDA_18V			EXYNOS5420_GPB3(0)
#define	GPIO_TSP_SCL_18V			EXYNOS5420_GPB3(1)
#define	GPIO_PEN_SDA_18V		EXYNOS5420_GPB3(4)
#define	GPIO_PEN_SCL_18V			EXYNOS5420_GPB3(5)
#define	GPIO_IF_PMIC_SDA			EXYNOS5420_GPB3(6)
#define	GPIO_IF_PMIC_SCL			EXYNOS5420_GPB3(7)
#define	GPIO_MHL_SDA_18V		EXYNOS5420_GPB4(0)
#define	GPIO_MHL_SCL_18V		EXYNOS5420_GPB4(1)

#define	GPIO_NAND_CLK			EXYNOS5420_GPC0(0)
#define	GPIO_NAND_CMD			EXYNOS5420_GPC0(1)
#define	GPIO_eMMC_EN			EXYNOS5420_GPC0(2)
#define	GPIO_NAND_D0			EXYNOS5420_GPC0(3)
#define	GPIO_NAND_D1			EXYNOS5420_GPC0(4)
#define	GPIO_NAND_D2			EXYNOS5420_GPC0(5)
#define	GPIO_NAND_D3			EXYNOS5420_GPC0(6)
#define	GPIO_EMMC_RCLK			EXYNOS5420_GPC0(7)

#define	GPIO_NAND_D4			EXYNOS5420_GPC3(0)
#define	GPIO_NAND_D5			EXYNOS5420_GPC3(1)
#define	GPIO_NAND_D6			EXYNOS5420_GPC3(2)
#define	GPIO_NAND_D7			EXYNOS5420_GPC3(3)

#define	GPIO_WLAN_SDIO_CLK		EXYNOS5420_GPC1(0)
#define	GPIO_WLAN_SDIO_CLK_AF	2
#define	GPIO_WLAN_SDIO_CMD		EXYNOS5420_GPC1(1)
#define	GPIO_WLAN_SDIO_CMD_AF	2
#define	GPIO_WLAN_SDIO_D0		EXYNOS5420_GPC1(3)
#define	GPIO_WLAN_SDIO_D0_AF	2
#define	GPIO_WLAN_SDIO_D1		EXYNOS5420_GPC1(4)
#define	GPIO_WLAN_SDIO_D1_AF	2
#define	GPIO_WLAN_SDIO_D2		EXYNOS5420_GPC1(5)
#define	GPIO_WLAN_SDIO_D2_AF	2
#define	GPIO_WLAN_SDIO_D3		EXYNOS5420_GPC1(6)
#define	GPIO_WLAN_SDIO_D3_AF	2

#define	GPIO_T_FLASH_CLK			EXYNOS5420_GPC2(0)
#define	GPIO_T_FLASH_CMD		EXYNOS5420_GPC2(1)
#define	GPIO_T_FLASH_D0			EXYNOS5420_GPC2(3)
#define	GPIO_T_FLASH_D1			EXYNOS5420_GPC2(4)
#define	GPIO_T_FLASH_D2			EXYNOS5420_GPC2(5)
#define	GPIO_T_FLASH_D3			EXYNOS5420_GPC2(6)

#define	GPIO_PEN_RESET_N_18V		EXYNOS5420_GPD1(1)
#define	GPIO_PEN_PDCT_18V		EXYNOS5420_GPD1(2)
#define	GPIO_PEN_FWE1_18V		EXYNOS5420_GPD1(3)
#define	GPIO_2TOUCH_SDA		EXYNOS5420_GPD1(4)
#define	GPIO_2TOUCH_SCL			EXYNOS5420_GPD1(5)
#define	GPIO_LCD_TE				EXYNOS5420_GPD1(7)

#define	GPIO_CAM_FLASH_EN		EXYNOS5420_GPE0(0)
#define	GPIO_CAM_FLASH_SET		EXYNOS5420_GPE0(1)
#define	GPIO_CAM_VT_STBY		EXYNOS5420_GPE0(2)
#define	GPIO_CAM_VT_nRST		EXYNOS5420_GPE0(4)
#define	GPIO_MAIN_CAM_RESET	EXYNOS5420_GPE0(5)

#define	GPIO_MAIN_CAM_SDA_18V	EXYNOS5420_GPF0(0)
#define	GPIO_MAIN_CAM_SCL_18V	EXYNOS5420_GPF0(1)
#define	GPIO_AF_SDA				EXYNOS5420_GPF0(2)
#define	GPIO_AF_SCL				EXYNOS5420_GPF0(3)
#define	GPIO_VT_CAM_SDA_18V		EXYNOS5420_GPF0(4)
#define	GPIO_VT_CAM_SCL_18V		EXYNOS5420_GPF0(5)
#define	GPIO_CAM_SPI_SCLK		EXYNOS5420_GPF1(0)
#define	GPIO_CAM_SPI_SSN		EXYNOS5420_GPF1(1)
#define	GPIO_CAM_SPI_MISO		EXYNOS5420_GPF1(2)
#define	GPIO_CAM_SPI_MOSI		EXYNOS5420_GPF1(3)
#define	GPIO_MMC01_EN			EXYNOS5420_GPF1(4)
#define	GPIO_MMC2_EN			EXYNOS5420_GPF1(5)
#define	GPIO_MIPI_18V_EN			EXYNOS5420_GPF1(6)
/* #define	GPIO_ERR_FG		EXYNOS5420_GPF1(7) */

#define	GPIO_MCU_nRST_18V		EXYNOS5420_GPG0(0)
#define	GPIO_YMU_LDO_EN		EXYNOS5420_GPG0(1)
#define	GPIO_FPGA_CDONE				EXYNOS5420_GPG0(2)
#define	GPIO_MHL_INT			EXYNOS5420_GPG0(3)
#define	GPIO_S_LED_I2C_SCL		EXYNOS5420_GPG0(4)
#define	GPIO_S_LED_I2C_SDA		EXYNOS5420_GPG0(5)
#define	GPIO_PCD_INT			EXYNOS5420_GPG0(6)
#define	GPIO_BT_EN				EXYNOS5420_GPG0(7)

#define	GPIO_IPC_SLAVE_WAKEUP	EXYNOS5420_GPG1(0)
#define	GPIO_NFC_FIRMWARE		EXYNOS5420_GPG1(1)
#define	GPIO_NFC_EN				EXYNOS5420_GPG1(2)
#define	GPIO_SUSPEND_REQUEST	EXYNOS5420_GPG1(3)
#define	GPIO_PHONE_ON			EXYNOS5420_GPG1(4)
#define	GPIO_AP_DUMP_INT		EXYNOS5420_GPG1(6)
#define	GPIO_ACTIVE_STATE		EXYNOS5420_GPG1(7)

#define	GPIO_FPGA_RST_N			EXYNOS5420_GPG2(0)
#define	GPIO_CORE_MODE			EXYNOS5420_GPG2(1)

#define	GPIO_HW_REV0			EXYNOS5420_GPH0(0)
#define	GPIO_HW_REV1			EXYNOS5420_GPH0(1)
#define	GPIO_HW_REV2			EXYNOS5420_GPH0(2)
#define	GPIO_HW_REV3			EXYNOS5420_GPH0(3)
#define	GPIO_FPGA_SPI_CLK		EXYNOS5420_GPH0(4)
#define	GPIO_CAM_MCLK			EXYNOS5420_GPH0(5)
#define	GPIO_FPGA_SPI_EN			EXYNOS5420_GPH0(6)
#define	GPIO_VTCAM_MCLK		EXYNOS5420_GPH0(7)

#define	GPIO_PSR_TE				EXYNOS5420_GPJ4(0)
#define	GPIO_RESET_REQ_N			EXYNOS5420_GPJ4(1)
#define	GPIO_FPGA_CRESET_B			EXYNOS5420_GPJ4(2)
#define	GPIO_MHL_RST			EXYNOS5420_GPJ4(3)

#define	GPIO_WACOM_SENSE		EXYNOS5420_GPX0(0)
#define	GPIO_HALL_SENSOR_INT	EXYNOS5420_GPX0(1)
#define	GPIO_VOL_UP				EXYNOS5420_GPX0(2)
#define	GPIO_VOL_DOWN			EXYNOS5420_GPX0(3)
#define	GPIO_IRDA_IRQ			EXYNOS5420_GPX0(4)
#define	GPIO_HOME_KEY			EXYNOS5420_GPX0(5)
#define	GPIO_GPS_HOST_WAKE		EXYNOS5420_GPX0(6)
#define	GPIO_GPS_HOST_WAKE_AF	0xF
#define	GPIO_AP_PMIC_IRQ			EXYNOS5420_GPX0(7)

#define	GPIO_GPS_PWR_EN			EXYNOS5420_GPX1(0)
#define	GPIO_CP_PMU_RST			EXYNOS5420_GPX1(1)
#define	GPIO_IPC_HOST_WAKEUP	EXYNOS5420_GPX1(2)
#define	GPIO_NFC_IRQ			EXYNOS5420_GPX1(3)
#define	GPIO_IF_PMIC_IRQ			EXYNOS5420_GPX1(4)
#define	GPIO_FUEL_ALERT			EXYNOS5420_GPX1(5)
#define	GPIO_TSP_nINT			EXYNOS5420_GPX1(6)
#define	GPIO_BTP_IRQ				EXYNOS5420_GPX1(7)

#define	GPIO_WLAN_HOST_WAKE	EXYNOS5420_GPX2(0)
#define	GPIO_WLAN_HOST_WAKE_AF	0xF
#define	GPIO_BT_HOST_WAKE		EXYNOS5420_GPX2(1)
#define	GPIO_nPOWER				EXYNOS5420_GPX2(2)
#define	GPIO_W_CHG_DET			EXYNOS5420_GPX2(3)
#define	GPIO_T_FLASH_DETECT		EXYNOS5420_GPX2(4)
#define	GPIO_EAR_SEND_END		EXYNOS5420_GPX2(5)
#define	GPIO_CP_DUMP_INT		EXYNOS5420_GPX2(6)
#define	GPIO_PHONE_ACTIVE		EXYNOS5420_GPX2(7)

#define	GPIO_AP_MCU_INT_18V		EXYNOS5420_GPX3(0)
#define	GPIO_BTP_RST_N			EXYNOS5420_GPX3(1)
#define	GPIO_BT_WAKE			EXYNOS5420_GPX3(2)
#define	GPIO_MCU_AP_INT_1_18V	EXYNOS5420_GPX3(3)
#define	GPIO_MCU_AP_INT_2_18V	EXYNOS5420_GPX3(4)
#define	GPIO_PEN_IRQ_18V			EXYNOS5420_GPX3(5)
#define	GPIO_PDA_ACTIVE			EXYNOS5420_GPX3(6)
#define	GPIO_HDMI_HPD			EXYNOS5420_GPX3(7)

#define	GPIO_MM_I2S_CLK			EXYNOS5420_GPZ(0)
#define	GPIO_MM_I2S_SYNC		EXYNOS5420_GPZ(2)
#define	GPIO_MM_I2S_DI			EXYNOS5420_GPZ(3)
#define	GPIO_MM_I2S_DO			EXYNOS5420_GPZ(4)

#define	GPIO_AP_JTAG_NTRST		EXYNOS5420_ETC0(0)
#define	GPIO_AP_JTAG_TMS		EXYNOS5420_ETC0(1)
#define	GPIO_AP_JTAG_TCK			EXYNOS5420_ETC0(2)
#define	GPIO_AP_JTAG_DI			EXYNOS5420_ETC0(3)
#define	GPIO_AP_JTAG_DO			EXYNOS5420_ETC0(4)

#define	GPIO_AP_N_RST_IN			EXYNOS5420_ETC6(0)
#define	GPIO_AP_JTAG_EXTRST		EXYNOS5420_ETC6(3)

#define GPIO_TF_EN			EXYNOS5420_GPY2(0)

#define	GPIO_FM_SCL_18V			EXYNOS5420_GPY6(3)
#define	GPIO_FM_SDA_18V			EXYNOS5420_GPY6(4)
#define	GPIO_FM_RST				EXYNOS5420_GPY6(5)
#define	GPIO_FM_INT				EXYNOS5420_GPY6(6)

#define	GPIO_CAM_SENSOR_A28V_VT_EN	EXYNOS5420_GPY7(0)
#define	GPIO_2TOUCH_INT			EXYNOS5420_GPY7(1)
#define	GPIO_VT_CAM_ID			EXYNOS5420_GPY7(2)
#define	GPIO_SENSOR_DET			EXYNOS5420_GPY7(3)
#define	GPIO_MLCD_RST			EXYNOS5420_GPY7(4)
#define	GPIO_USB30_REDRIVER_EN	EXYNOS5420_GPY7(5)
#define	GPIO_MICBIAS_EN			EXYNOS5420_GPY7(6)
#define	GPIO_WLAN_EN			EXYNOS5420_GPY7(7)
/* __MACH_GPIO_EXYNOS5420_UNIVERSAL_EVT0_H */
#endif
