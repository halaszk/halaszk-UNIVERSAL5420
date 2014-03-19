/* linux/drivers/video/backlight/s6tnmr7_mipi_lcd.c
 *
 * Samsung SoC MIPI LCD driver.
 *
 * Copyright (c) 2012 Samsung Electronics
 *
 * Haowei Li, <haowei.li@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/backlight.h>
#include <linux/lcd.h>
#include <linux/rtc.h>
#include <linux/reboot.h>
#include <linux/gpio.h>

#include <video/mipi_display.h>
#include <plat/dsim.h>
#include <plat/mipi_dsi.h>
#include <plat/gpio-cfg.h>
#include <asm/system_info.h>

#include "s6tnmr7_gamma.h"
#include "s6tnmr7_param.h"

#include "dynamic_aid_s6tnmr7.h"

#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		255
#define DEFAULT_BRIGHTNESS		162

#define POWER_IS_ON(pwr)		(pwr <= FB_BLANK_NORMAL)

#define MAX_GAMMA			350
#define DEFAULT_GAMMA_LEVEL		GAMMA_143CD

#define LDI_ID_REG			0x04
#define LDI_ID_LEN			3
#define LDI_ADDR_BASE			0xB1
#define LDI_MTPR_REG			0x2100
#define LDI_MTPG_REG			0x2180
#define LDI_MTPB_REG			0x2200
#define MTP_VMAX			11
#define LDI_MTP_LEN			(MTP_VMAX * 3)
#define LDI_ELVSS_REG			0xB6
#define LDI_ELVSS_LEN			17

#define LDI_COORDINATE_REG		0xA1
#define LDI_COORDINATE_LEN		4

#define LDI_BURST_SIZE		128
#define LDI_PARAM_MSB		0xB1
#define LDI_MDNIE_SIZE		136
#define MDNIE_FIRST_SIZE	82
#define MDNIE_SECOND_SIZE	54

#define LDI_TSET_REG			0xB8
#define LDI_TSET_LEN			8	/* REG address + 7 para */

#define LDI_GAMMA_REG		0x83

#ifdef SMART_DIMMING_DEBUG
#define smtd_dbg(format, arg...)	printk(format, ##arg)
#else
#define smtd_dbg(format, arg...)
#endif

struct lcd_info {
	unsigned int			bl;
	unsigned int			auto_brightness;
	unsigned int			acl_enable;
	unsigned int			siop_enable;
	unsigned int			current_acl;
	unsigned int			current_bl;
	unsigned int			current_elvss;
	unsigned int			current_psre;
	unsigned int			current_tset;
	unsigned int			ldi_enable;
	unsigned int			power;
	struct mutex			lock;
	struct mutex			bl_lock;

	struct device			*dev;
	struct lcd_device		*ld;
	struct backlight_device		*bd;
	unsigned char			id[LDI_ID_LEN];
	unsigned char			**gamma_table;
	unsigned char			**elvss_table[ELVSS_TABLE_NUM];
	struct dynamic_aid_param_t daid;
	unsigned char			aor[GAMMA_MAX][ARRAY_SIZE(SEQ_AOR_CONTROL)];
	unsigned int			connected;

	unsigned char			**tset_table;
	int				temperature;

	unsigned int			coordinate[2];
	unsigned int			partial_range[2];

	struct mipi_dsim_device		*dsim;
};

static const unsigned int candela_table[GAMMA_MAX] = {
	2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
	15, 16, 17, 19, 20, 21, 22, 24, 25, 27,
	29, 30, 32, 34, 37, 39, 41, 44, 47, 50,
	53, 56, 60, 64, 68, 72, 77, 82, 87, 93,
	98, 105, 111, 119, 126, 134, 143, 152, 162, 172,
	183, 195, 207, 220, 234, 249, 265, 282, 300, MAX_GAMMA-1,
};

static struct lcd_info *g_lcd;
static int update_brightness(struct lcd_info *lcd, u8 force);


static int s6tnmr7_write(struct lcd_info *lcd, const u8 *seq, u32 len)
{
	int ret;
	int retry;
	u8 cmd;

	if (!lcd->connected)
		return -EINVAL;

	mutex_lock(&lcd->lock);

	if (len > 2)
		cmd = MIPI_DSI_DCS_LONG_WRITE;
	else if (len == 2)
		cmd = MIPI_DSI_DCS_SHORT_WRITE_PARAM;
	else if (len == 1)
		cmd = MIPI_DSI_DCS_SHORT_WRITE;
	else {
		ret = -EINVAL;
		goto write_err;
	}

	retry = 5;
write_data:
	if (!retry) {
		dev_err(&lcd->ld->dev, "%s failed: exceed retry count\n", __func__);
		/* print_reg_pm_disp1(); */
		goto write_err;
	}
	ret = s5p_mipi_dsi_wr_data(lcd->dsim, cmd, seq, len);
	if (ret != len) {
		dev_dbg(&lcd->ld->dev, "mipi_write failed retry ..\n");
		retry--;
		goto write_data;
	}

write_err:
	mutex_unlock(&lcd->lock);
	return ret;
}

static int s6tnmr7_read(struct lcd_info *lcd, u8 addr, u8 *buf, u32 len)
{
	int ret = 0;
	u8 cmd;
	int retry;

	if (!lcd->connected)
		return -EINVAL;

	mutex_lock(&lcd->lock);
	if (len > 2)
		cmd = MIPI_DSI_DCS_READ;
	else if (len == 2)
		cmd = MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM;
	else if (len == 1)
		cmd = MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM;
	else {
		ret = -EINVAL;
		goto read_err;
	}
	retry = 5;
read_data:
	if (!retry) {
		dev_err(&lcd->ld->dev, "%s failed: exceed retry count\n", __func__);
		goto read_err;
	}
	ret = s5p_mipi_dsi_rd_data(lcd->dsim, cmd, addr, len, buf, 0);
	if (ret != len) {
		dev_dbg(&lcd->ld->dev, "mipi_read failed retry ..\n");
		retry--;
		goto read_data;
	}
read_err:
	mutex_unlock(&lcd->lock);
	return ret;
}

static void s6tnmr7_read_id(struct lcd_info *lcd, u8 *buf)
{
	int ret;

	ret = s6tnmr7_read(lcd, LDI_ID_REG, buf, LDI_ID_LEN);
	if (ret < 1) {
		lcd->connected = 0;
		dev_info(&lcd->ld->dev, "panel is not connected well\n");
	}


}

static int s6tnmr7_tsp_te_enable(struct lcd_info *lcd, int onoff)
{
	int ret;

	ret = s6tnmr7_write(lcd, SEQ_TSP_TE_B0, ARRAY_SIZE(SEQ_TSP_TE_B0));
	if (ret < ARRAY_SIZE(SEQ_TSP_TE_B0))
		goto te_err;

	ret = s6tnmr7_write(lcd, SEQ_TSP_TE_EN, ARRAY_SIZE(SEQ_TSP_TE_EN));
	if (ret < ARRAY_SIZE(SEQ_TSP_TE_EN))
		goto te_err;

	return ret;

te_err:
	dev_err(&lcd->ld->dev, "%s onoff=%d fail\n", __func__, onoff);
	return ret;
}

static int s6tnmr7_mdnie_enable(struct lcd_info *lcd, int onoff)
{
	int ret;

	ret = s6tnmr7_write(lcd, SEQ_MDNIE_EN_B0, ARRAY_SIZE(SEQ_MDNIE_EN_B0));
	if (ret < ARRAY_SIZE(SEQ_MDNIE_EN_B0))
		goto enable_err;

	ret = s6tnmr7_write(lcd, SEQ_MDNIE_EN, ARRAY_SIZE(SEQ_MDNIE_EN));
	if (ret < ARRAY_SIZE(SEQ_MDNIE_EN))
		goto enable_err;

	return ret;

enable_err:
	dev_err(&lcd->ld->dev, "%s onoff=%d fail\n", __func__, onoff);
	return ret;
}

static void s6tnmr7_read_mdnie(struct lcd_info *lcd)
{
	int ret;
	int i;
	u8 buf[LDI_MDNIE_SIZE];

	dev_info(&lcd->ld->dev, "%s\n", __func__);

	s6tnmr7_write(lcd, SEQ_MDNIE_START_B0, ARRAY_SIZE(SEQ_MDNIE_START_B0));
	msleep(120);

	ret = s6tnmr7_read(lcd, 0xBA, buf, LDI_MDNIE_SIZE);
	if (ret < 1) {
		dev_info(&lcd->ld->dev, "panel is not connected well\n");
	}

	for (i=0; i < 136; i++)
		pr_info(" %02d = 0x%02x\n", i, buf[i]);

}

static int s6tnmr7_write_mdnie(struct lcd_info *lcd,
		const unsigned short *seq, int size)
{
	int ret;
	unsigned char buf[LDI_BURST_SIZE];
	int send_len;
	int i;

	buf[0] = 0xB0;
	buf[1] = (unsigned char) (seq[0] & 0xff);
	dev_dbg(&lcd->ld->dev, "[0x%02x][0x%02x]\n", buf[0], buf[1]);

	ret = s6tnmr7_write(lcd, buf, 2);
	if (ret < 2)
		return -EINVAL;

	buf[0] = ((seq[0] >> 8) & 0xff) + LDI_PARAM_MSB;
	for ( i = 0; i < size; i++)
		buf[i+1] = (unsigned char)(seq[i*2+1] & 0xff);

	send_len = size + 1; /* addr data */
	for (i=0; i < send_len; i++)
		dev_dbg(&lcd->ld->dev, "%s : [%02d] = 0x%02x\n", __func__, i, buf[i]);

	ret = s6tnmr7_write(lcd, buf, send_len);

	if (ret < send_len)
		return -EINVAL;

	return size;
}

int mdnie_lite_write(const unsigned short *seq, int size)
{
	struct lcd_info *lcd = g_lcd;
	int ret;

	if (!lcd->connected)
		return -EINVAL;

	if (IS_ERR_OR_NULL(seq)) {
		dev_err(&lcd->ld->dev, "mdnie sequence is null\n");
		return -EPERM;
	}

	if (LDI_MDNIE_SIZE != size) {
		dev_err(&lcd->ld->dev, "mdnie sequence size error (%d)\n", size);
		return -EPERM;
	}

	ret = s6tnmr7_write_mdnie(lcd, seq, MDNIE_FIRST_SIZE);
	if (ret != MDNIE_FIRST_SIZE) {
		dev_err(&lcd->ld->dev, "MDNIE first param write error\n");
		return -EINVAL;
	}

	msleep(17*2); /* wait 1 frame */

	ret = s6tnmr7_write_mdnie(lcd,
		seq + MDNIE_FIRST_SIZE * 2, MDNIE_SECOND_SIZE);

	if (ret != MDNIE_SECOND_SIZE) {
		dev_err(&lcd->ld->dev, "MDNIE second param write error\n");
		return -EINVAL;
	}

	return size;
}

static int s6tnmr7_ldi_init(struct lcd_info *lcd)
{
	int ret = 0;

	lcd->connected = 1;

	msleep(120);

	s6tnmr7_read_id(lcd, lcd->id);
	dev_info(&lcd->ld->dev," %s : id [%x] [%x] [%x] \n", __func__,
		lcd->id[0], lcd->id[1], lcd->id[2]);

	update_brightness(lcd, 1);

#if defined(CONFIG_FB_S5P_MDNIE_LITE)
	s6tnmr7_mdnie_enable(lcd, 1);
#endif
	s6tnmr7_tsp_te_enable(lcd, 1);
	msleep(120);

	return ret;
}

static int s6tnmr7_ldi_enable(struct lcd_info *lcd)
{
	int ret = 0;

	s6tnmr7_write(lcd, SEQ_DISPLAY_ON, ARRAY_SIZE(SEQ_DISPLAY_ON));

	return ret;
}

static int s6tnmr7_ldi_disable(struct lcd_info *lcd)
{
	int ret = 0;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	s6tnmr7_write(lcd, SEQ_DISPLAY_OFF, ARRAY_SIZE(SEQ_DISPLAY_OFF));

	msleep(35);

	/* after display off there is okay to send the commands via MIPI DSI Command
	because we don't need to worry about screen blinking. */
	s6tnmr7_write(lcd, SEQ_SLEEP_IN, ARRAY_SIZE(SEQ_SLEEP_IN));

	msleep(125);
	dev_info(&lcd->ld->dev, "- %s\n", __func__);

	return ret;
}

static int s6tnmr7_power_on(struct lcd_info *lcd)
{
	int ret;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	ret = s6tnmr7_ldi_init(lcd);
	if (ret) {
		dev_err(&lcd->ld->dev, "failed to initialize ldi.\n");
		goto err;
	}

	ret = s6tnmr7_ldi_enable(lcd);
	if (ret) {
		dev_err(&lcd->ld->dev, "failed to enable ldi.\n");
		goto err;
	}

	lcd->ldi_enable = 1;

	/* update_brightness(lcd, 1); */

	dev_info(&lcd->ld->dev, "- %s\n", __func__);
err:
	return ret;
}

static int s6tnmr7_power_off(struct lcd_info *lcd)
{
	int ret;

	dev_info(&lcd->ld->dev, "+ %s\n", __func__);

	lcd->ldi_enable = 0;

	ret = s6tnmr7_ldi_disable(lcd);

	dev_info(&lcd->ld->dev, "- %s\n", __func__);

	return ret;
}
static int s6tnmr7_power(struct lcd_info *lcd, int power)
{
	int ret = 0;

	if (POWER_IS_ON(power) && !POWER_IS_ON(lcd->power))
		ret = s6tnmr7_power_on(lcd);
	else if (!POWER_IS_ON(power) && POWER_IS_ON(lcd->power))
		ret = s6tnmr7_power_off(lcd);

	if (!ret)
		lcd->power = power;

	return ret;
}

static int s6tnmr7_set_power(struct lcd_device *ld, int power)
{
	struct lcd_info *lcd = lcd_get_data(ld);

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
		power != FB_BLANK_NORMAL) {
		dev_err(&lcd->ld->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	return s6tnmr7_power(lcd, power);
}

static int s6tnmr7_get_power(struct lcd_device *ld)
{
	struct lcd_info *lcd = lcd_get_data(ld);

	return lcd->power;
}

static int s6tnmr7_check_fb(struct lcd_device *ld, struct fb_info *fb)
{
	return 0;
}

static int s6tnmr7_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

#if 0
static void s6tnmr7_read_coordinate(struct lcd_info *lcd)
{
	int ret;
	unsigned char buf[LDI_COORDINATE_LEN] = {0,};

	ret = s6tnmr7_read(lcd, LDI_COORDINATE_REG, buf, LDI_COORDINATE_LEN);

	if (ret < 1)
		dev_err(&lcd->ld->dev, "%s failed\n", __func__);

	lcd->coordinate[0] = buf[0] << 8 | buf[1];	/* X */
	lcd->coordinate[1] = buf[2] << 8 | buf[3];	/* Y */
}
#endif

static int s6tnmr7_read_mtp(struct lcd_info *lcd, u8 *buf)
{
	int ret = 0, i;
	int mtp_addr[3];
	u8 mtp_offset[2] = {0xB0,};

	mtp_addr[0] = LDI_MTPR_REG + (LDI_ADDR_BASE << 8);
	mtp_addr[1] = LDI_MTPG_REG + (LDI_ADDR_BASE << 8);
	mtp_addr[2] = LDI_MTPB_REG + (LDI_ADDR_BASE << 8);

	for (i = 0; i < CI_MAX; i++) {
		mtp_offset[1] = (u8) (mtp_addr[i] & 0xff);
		s6tnmr7_write(lcd, mtp_offset, 2);

		ret = s6tnmr7_read(lcd, (u8) (mtp_addr[i] >> 8),
			buf + i*MTP_VMAX, MTP_VMAX);

		if (ret < 1) {
			dev_err(&lcd->ld->dev, "%s failed\n", __func__);
			return ret;
		}
	}

	for (i = 0; i < LDI_MTP_LEN ; i++)
		smtd_dbg("%02dth mtp value is %02x\n", i+1, (int)buf[i]);

	return LDI_MTP_LEN;
}

static int get_backlight_level_from_brightness(int brightness)
{
	int backlightlevel;

	switch (brightness) {
	case 0 ... 2:
		backlightlevel = GAMMA_2CD;
		break;
	case 3:
		backlightlevel = GAMMA_3CD;
		break;
	case 4:
		backlightlevel = GAMMA_4CD;
		break;
	case 5:
		backlightlevel = GAMMA_5CD;
		break;
	case 6:
		backlightlevel = GAMMA_6CD;
		break;
	case 7:
		backlightlevel = GAMMA_7CD;
		break;
	case 8:
		backlightlevel = GAMMA_8CD;
		break;
	case 9:
		backlightlevel = GAMMA_9CD;
		break;
	case 10:
		backlightlevel = GAMMA_10CD;
		break;
	case 11:
		backlightlevel = GAMMA_11CD;
		break;
	case 12:
		backlightlevel = GAMMA_12CD;
		break;
	case 13:
		backlightlevel = GAMMA_13CD;
		break;
	case 14:
		backlightlevel = GAMMA_14CD;
		break;
	case 15:
		backlightlevel = GAMMA_15CD;
		break;
	case 16:
		backlightlevel = GAMMA_16CD;
		break;
	case 17 ... 18:
		backlightlevel = GAMMA_17CD;
		break;
	case 19:
		backlightlevel = GAMMA_19CD;
		break;
	case 20:
		backlightlevel = GAMMA_20CD;
		break;
	case 21:
		backlightlevel = GAMMA_21CD;
		break;
	case 22 ... 23:
		backlightlevel = GAMMA_22CD;
		break;
	case 24:
		backlightlevel = GAMMA_24CD;
		break;
	case 25 ... 26:
		backlightlevel = GAMMA_25CD;
		break;
	case 27 ... 28:
		backlightlevel = GAMMA_27CD;
		break;
	case 29:
		backlightlevel = GAMMA_29CD;
		break;
	case 30 ... 31:
		backlightlevel = GAMMA_30CD;
		break;
	case 32 ... 33:
		backlightlevel = GAMMA_32CD;
		break;
	case 34 ... 36:
		backlightlevel = GAMMA_34CD;
		break;
	case 37 ... 38:
		backlightlevel = GAMMA_37CD;
		break;
	case 39 ... 40:
		backlightlevel = GAMMA_39CD;
		break;
	case 41 ... 43:
		backlightlevel = GAMMA_41CD;
		break;
	case 44 ... 46:
		backlightlevel = GAMMA_44CD;
		break;
	case 47 ... 49:
		backlightlevel = GAMMA_47CD;
		break;
	case 50 ... 52:
		backlightlevel = GAMMA_50CD;
		break;
	case 53 ... 55:
		backlightlevel = GAMMA_53CD;
		break;
	case 56 ... 59:
		backlightlevel = GAMMA_56CD;
		break;
	case 60 ... 63:
		backlightlevel = GAMMA_60CD;
		break;
	case 64 ... 67:
		backlightlevel = GAMMA_64CD;
		break;
	case 68 ... 71:
		backlightlevel = GAMMA_68CD;
		break;
	case 72 ... 76:
		backlightlevel = GAMMA_72CD;
		break;
	case 77 ... 81:
		backlightlevel = GAMMA_77CD;
		break;
	case 82 ... 86:
		backlightlevel = GAMMA_82CD;
		break;
	case 87 ... 92:
		backlightlevel = GAMMA_87CD;
		break;
	case 93 ... 97:
		backlightlevel = GAMMA_93CD;
		break;
	case 98 ... 104:
		backlightlevel = GAMMA_98CD;
		break;
	case 105 ... 110:
		backlightlevel = GAMMA_105CD;
		break;
	case 111 ... 118:
		backlightlevel = GAMMA_111CD;
		break;
	case 119 ... 125:
		backlightlevel = GAMMA_119CD;
		break;
	case 126 ... 133:
		backlightlevel = GAMMA_126CD;
		break;
	case 134 ... 142:
		backlightlevel = GAMMA_134CD;
		break;
	case 143 ... 149:
		backlightlevel = GAMMA_143CD;
		break;
	case 150 ... 161:
		backlightlevel = GAMMA_152CD;
		break;
	case 162 ... 171:
		backlightlevel = GAMMA_162CD;
		break;
	case 172 ... 181:
		backlightlevel = GAMMA_172CD;
		break;
	case 182 ... 193:
		backlightlevel = GAMMA_183CD;
		break;
	case 194 ... 205:
		backlightlevel = GAMMA_195CD;
		break;
	case 206 ... 218:
		backlightlevel = GAMMA_207CD;
		break;
	case 219 ... 229:
		backlightlevel = GAMMA_220CD;
		break;
	case 230 ... 237:
		backlightlevel = GAMMA_234CD;
		break;
	case 238 ... 241:
		backlightlevel = GAMMA_249CD;
		break;
	case 242 ... 244:
		backlightlevel = GAMMA_265CD;
		break;
	case 245 ... 247:
		backlightlevel = GAMMA_282CD;
		break;
	case 248 ... 249:
		backlightlevel = GAMMA_300CD;
		break;
	case 250 ... 255:
		backlightlevel = GAMMA_300CD;
		break;
	default:
		backlightlevel = DEFAULT_GAMMA_LEVEL;
		break;
	}

	return backlightlevel;
}

static int s6tnmr7_gamma_ctl(struct lcd_info *lcd)
{
	s6tnmr7_write(lcd, lcd->gamma_table[lcd->bl], GAMMA_PARAM_SIZE);

	return 0;
}

static int s6tnmr7_aid_parameter_ctl(struct lcd_info *lcd, u8 force)
{
	if (force)
		goto aid_update;
	else if (lcd->aor[lcd->bl][1] !=  lcd->aor[lcd->current_bl][1])
		goto aid_update;
	else if (lcd->aor[lcd->bl][2] !=  lcd->aor[lcd->current_bl][2])
		goto aid_update;
	else
		goto exit;

aid_update:
	s6tnmr7_write(lcd, lcd->aor[lcd->bl], AID_PARAM_SIZE);

exit:
	s6tnmr7_write(lcd, SEQ_GLOBAL_PARAM_47RD, ARRAY_SIZE(SEQ_GLOBAL_PARAM_47RD));
	s6tnmr7_write(lcd, SEQ_GAMMA_UPDATE, ARRAY_SIZE(SEQ_GAMMA_UPDATE));

	return 0;
}

static int s6tnmr7_set_acl(struct lcd_info *lcd, u8 force)
{
	int ret = 0, level = 0;

	level = ACL_STATUS_25P;

	if (lcd->siop_enable)
		goto acl_update;

	if (!lcd->acl_enable)
		level = ACL_STATUS_0P;

acl_update:
	if (force || lcd->current_acl != ACL_CUTOFF_TABLE[level][1]) {
		s6tnmr7_write(lcd, SEQ_GLOBAL_PARAM_47RD, ARRAY_SIZE(SEQ_GLOBAL_PARAM_47RD));
		ret = s6tnmr7_write(lcd, ACL_CUTOFF_TABLE[level], ACL_PARAM_SIZE);
		lcd->current_acl = ACL_CUTOFF_TABLE[level][1];
		dev_info(&lcd->ld->dev, "acl: %d, auto_brightness: %d\n", lcd->current_acl, lcd->auto_brightness);
	}

	if (!ret)
		ret = -EPERM;

	return ret;
}

static int s6tnmr7_set_elvss(struct lcd_info *lcd, u8 force)
{
	int ret = 0, elvss_level = 0, elvss;
	u32 candela = candela_table[lcd->bl];

	switch (candela) {
	case 0 ... 105:
		elvss_level = ELVSS_STATUS_105;
		break;
	case 106 ... 111:
		elvss_level = ELVSS_STATUS_111;
		break;
	case 112 ... 119:
		elvss_level = ELVSS_STATUS_119;
		break;
	case 120 ... 126:
		elvss_level = ELVSS_STATUS_126;
		break;
	case 127 ... 134:
		elvss_level = ELVSS_STATUS_134;
		break;
	case 135 ... 143:
		elvss_level = ELVSS_STATUS_143;
		break;
	case 144 ... 152:
		elvss_level = ELVSS_STATUS_152;
		break;
	case 153 ... 162:
		elvss_level = ELVSS_STATUS_162;
		break;
	case 163 ... 172:
		elvss_level = ELVSS_STATUS_172;
		break;
	case 173 ... 183:
		elvss_level = ELVSS_STATUS_183;
		break;
	case 184 ... 195:
		elvss_level = ELVSS_STATUS_195;
		break;
	case 196 ... 207:
		elvss_level = ELVSS_STATUS_207;
		break;
	case 208 ... 220:
		elvss_level = ELVSS_STATUS_220;
		break;
	case 221 ... 234:
		elvss_level = ELVSS_STATUS_234;
		break;
	case 235 ... 249:
		elvss_level = ELVSS_STATUS_249;
		break;
	case 250 ... 265:
		elvss_level = ELVSS_STATUS_265;
		break;
	case 266 ... 282:
		elvss_level = ELVSS_STATUS_282;
		break;
	case 283 ... 299:
		elvss_level = ELVSS_STATUS_300;
		break;
	default:
		elvss_level = ELVSS_STATUS_300;
		break;
	}

	elvss = lcd->elvss_table[lcd->acl_enable][elvss_level][1];

	if (force || lcd->elvss_table[lcd->acl_enable][lcd->current_elvss][1] != elvss) {
		ret = s6tnmr7_write(lcd, SEQ_GLOBAL_PARAM_53RD, ARRAY_SIZE(SEQ_GLOBAL_PARAM_53RD));
		ret = s6tnmr7_write(lcd, lcd->elvss_table[lcd->acl_enable][elvss_level], ELVSS_PARAM_SIZE);
		lcd->current_elvss = elvss_level;

		dev_dbg(&lcd->ld->dev, "elvss: %d, %d, %x\n", lcd->acl_enable, lcd->current_elvss,
			lcd->elvss_table[lcd->acl_enable][lcd->current_elvss][1]);
	}
	if (!ret) {
		ret = -EPERM;
		goto elvss_err;
	}

elvss_err:
	return ret;
}

static int s6tnmr7_set_tset(struct lcd_info *lcd, u8 force)
{
	int ret = 0, tset_level = 0;

	switch (lcd->temperature) {
	case 1:
		tset_level = TSET_25_DEGREES;
		break;
	case 0:
	case -19:
		tset_level = TSET_MINUS_0_DEGREES;
		break;
	case -20:
		tset_level = TSET_MINUS_20_DEGREES;
		break;
	}

	if (force || lcd->current_tset != tset_level) {
		s6tnmr7_write(lcd, SEQ_GLOBAL_PARAM_TSET, ARRAY_SIZE(SEQ_GLOBAL_PARAM_TSET));
		ret = s6tnmr7_write(lcd, lcd->tset_table[tset_level], 2);
		lcd->current_tset = tset_level;
		dev_info(&lcd->ld->dev, "tset: %d\n", lcd->current_tset);
	}

	if (!ret) {
		ret = -EPERM;
		goto err;
	}

err:
	return ret;
}

void init_dynamic_aid(struct lcd_info *lcd)
{
	lcd->daid.vreg = VREG_OUT_X1000;
	lcd->daid.vref_h = VREFH_OUT_X100000;
	lcd->daid.iv_tbl = index_voltage_table;
	lcd->daid.iv_max = IV_MAX;
	lcd->daid.mtp = kzalloc(IV_MAX * CI_MAX * sizeof(int), GFP_KERNEL);
	lcd->daid.gamma_default = gamma_default;
	lcd->daid.formular = gamma_formula;
	lcd->daid.vt_voltage_value = vt_voltage_value;

	lcd->daid.ibr_tbl = index_brightness_table;
	lcd->daid.ibr_max = IBRIGHTNESS_MAX;
	lcd->daid.gc_tbls = gamma_curve_tables;
	lcd->daid.gc_lut = gamma_curve_lut;
	lcd->daid.br_base = brightness_base_table;
	lcd->daid.offset_gra = offset_gradation;
	lcd->daid.offset_color = offset_color;
}

static void init_mtp_data(struct lcd_info *lcd, const u8 *mtp_data)
{
	int i, c, j;
	int mtp_val;
	int *mtp;
	int mtp_v0[3];

	mtp = lcd->daid.mtp;


	for (c = 0; c < CI_MAX ; c++) {
		for (i = IV_11, j = 0; i < IV_MAX; i++, j++)
			mtp[i*CI_MAX + c] = mtp_data[MTP_VMAX*c + j];

		mtp[IV_3*CI_MAX + c] = mtp_data[MTP_VMAX*c + j++];
		mtp_v0[c] = mtp_data[MTP_VMAX*c + j++];
		mtp[IV_VT*CI_MAX + c] = mtp_data[MTP_VMAX*c + j++];
	}

	for (c = 0; c < CI_MAX ; c++) {
		for (i = IV_3, j = 0; i <= IV_203; i++, j++)
			if (mtp[i*CI_MAX + c] & 0x80) {
				mtp[i*CI_MAX + c] = mtp[i*CI_MAX + c] & 0x7f;
				mtp[i*CI_MAX + c] *= (-1);
			}
		if (mtp_v0[c] & 0x80)
			mtp[IV_255*CI_MAX + c] *= (-1);
	}

	for (i = 0, j = 0; i <= IV_MAX; i++)
		for (c=0; c<CI_MAX ; c++, j++)
			smtd_dbg("mtp_data[%d] = %d\n",j, mtp_data[j]);

	for (i = 0, j = 0; i < IV_MAX; i++)
		for (c=0; c<CI_MAX ; c++, j++)
			smtd_dbg("mtp[%d] = %d\n",j, mtp[j]);
}

static int init_gamma_table(struct lcd_info *lcd , const u8 *mtp_data)
{
	int i, c, j, v;
	int ret = 0;
	int *pgamma;
	int **gamma;
	unsigned char	value;

	/* allocate memory for local gamma table */
	gamma = kzalloc(IBRIGHTNESS_MAX * sizeof(int *), GFP_KERNEL);
	if (!gamma) {
		pr_err("failed to allocate gamma table\n");
		ret = -ENOMEM;
		goto err_alloc_gamma_table;
	}

	for (i = 0; i < IBRIGHTNESS_MAX; i++) {
		gamma[i] = kzalloc(IV_MAX*CI_MAX * sizeof(int), GFP_KERNEL);
		if (!gamma[i]) {
			pr_err("failed to allocate gamma\n");
			ret = -ENOMEM;
			goto err_alloc_gamma;
		}
	}

	/* allocate memory for gamma table */
	lcd->gamma_table = kzalloc(GAMMA_MAX * sizeof(u8 *), GFP_KERNEL);
	if (!lcd->gamma_table) {
		pr_err("failed to allocate gamma table 2\n");
		ret = -ENOMEM;
		goto err_alloc_gamma_table2;
	}

	for (i = 0; i < GAMMA_MAX; i++) {
		lcd->gamma_table[i] = kzalloc(GAMMA_PARAM_SIZE * sizeof(u8), GFP_KERNEL);
		if (!lcd->gamma_table[i]) {
			pr_err("failed to allocate gamma 2\n");
			ret = -ENOMEM;
			goto err_alloc_gamma2;
		}
		lcd->gamma_table[i][0] = LDI_GAMMA_REG;
	}

	/* calculate gamma table */
	init_mtp_data(lcd, mtp_data);
	dynamic_aid(lcd->daid, gamma);

	/* relocate gamma order */
	for (i = 0; i < GAMMA_MAX - 1; i++) {
		/* Brightness table */
		for (c = 0, j = 1; c < CI_MAX ; c++, pgamma++) {
			for (v = IV_11; v < IV_MAX; v++) {
				pgamma = &gamma[i][v * CI_MAX + c];
				value = (char)((*pgamma) & 0xff);
				lcd->gamma_table[i][j++] = value;
			}
			pgamma = &gamma[i][IV_3 * CI_MAX + c];
			value = (char)((*pgamma) & 0xff);
			lcd->gamma_table[i][j++] = value;

			pgamma = &gamma[i][IV_255 * CI_MAX + c];
			value = (*pgamma & 0x100) ? 0x80 : 0x00;
			lcd->gamma_table[i][j++] = value;

			pgamma = &gamma[i][IV_VT * CI_MAX + c];
			value = (char)((*pgamma) & 0xff);
			lcd->gamma_table[i][j++] = value;
		}

		for (v = 0; v < GAMMA_PARAM_SIZE; v++)
			smtd_dbg("%d ", lcd->gamma_table[i][v]);
		smtd_dbg("\n");
	}

	/* free local gamma table */
	for (i = 0; i < IBRIGHTNESS_MAX; i++)
		kfree(gamma[i]);
	kfree(gamma);

	return 0;

err_alloc_gamma2:
	while (i > 0) {
		kfree(lcd->gamma_table[i-1]);
		i--;
	}
	kfree(lcd->gamma_table);
err_alloc_gamma_table2:
	i = IBRIGHTNESS_MAX;
err_alloc_gamma:
	while (i > 0) {
		kfree(gamma[i-1]);
		i--;
	}
	kfree(gamma);
err_alloc_gamma_table:
	return ret;
}

static int init_aid_dimming_table(struct lcd_info *lcd)
{
	int i, j;

	pSEQ_AOR_CONTROL = SEQ_AOR_CONTROL;


	for (i = 0; i < GAMMA_MAX; i++)
		memcpy(lcd->aor[i], pSEQ_AOR_CONTROL, ARRAY_SIZE(SEQ_AOR_CONTROL));

	for (i = 0; i < GAMMA_MAX -1; i++) {
		lcd->aor[i][1] = aor_cmd[i][1];
		lcd->aor[i][2] = aor_cmd[i][2];
	}

	for (i = 0; i < GAMMA_MAX; i++) {
		for (j = 0; j < ARRAY_SIZE(SEQ_AOR_CONTROL); j++)
			smtd_dbg("%02X ", lcd->aor[i][j]);
		smtd_dbg("\n");
	}

	return 0;
}

static int init_elvss_table(struct lcd_info *lcd, u8 *elvss_data)
{
	int i, j, k, ret;

	pELVSS_TABLE = ELVSS_TABLE;

	for (k = 0; k < ELVSS_TABLE_NUM; k++) {
		lcd->elvss_table[k] = kzalloc(ELVSS_STATUS_MAX * sizeof(u8 *), GFP_KERNEL);

		if (!lcd->elvss_table[k]) {
			pr_err("failed to allocate elvss table\n");
			ret = -ENOMEM;
			goto err_alloc_elvss_table;
		}
	}

	for (k = 0; k < ELVSS_TABLE_NUM; k++) {
		for (i = 0; i < ELVSS_STATUS_MAX; i++) {
			lcd->elvss_table[k][i] = kzalloc(ELVSS_PARAM_SIZE * sizeof(u8), GFP_KERNEL);
			if (!lcd->elvss_table[k][i]) {
				pr_err("failed to allocate elvss\n");
				ret = -ENOMEM;
				goto err_alloc_elvss;
			}

			lcd->elvss_table[k][i][0] = 0xBB;
			lcd->elvss_table[k][i][1] = pELVSS_TABLE[i][k];
		}

		for (i = 0; i < ELVSS_STATUS_MAX; i++) {
			for (j = 0; j < ELVSS_PARAM_SIZE; j++)
				smtd_dbg("0x%02x, ", lcd->elvss_table[k][i][j]);
			smtd_dbg("\n");
		}
	}

	return 0;

err_alloc_elvss:
	/* should be kfree elvss with k */
	while (k >= 0) {
		while (i > 0)
			kfree(lcd->elvss_table[k][--i]);

		i = ELVSS_STATUS_MAX;
		k--;
	}
	k = ELVSS_TABLE_NUM;
err_alloc_elvss_table:
	while (k > 0)
		kfree(lcd->elvss_table[--k]);

	return ret;
}

static int init_tset_table(struct lcd_info *lcd)
{

	int i, j, ret;
	return 0;
#if 0

	lcd->tset_table = kzalloc(TSET_STATUS_MAX * sizeof(u8 *), GFP_KERNEL);
	if (!lcd->tset_table) {
		pr_err("failed to allocate tset table\n");
		ret = -ENOMEM;
		goto err_alloc_tset_table;
	}

	for (i = 0; i < TSET_STATUS_MAX; i++) {
		lcd->tset_table[i] = kzalloc(LDI_TSET_LEN * sizeof(u8), GFP_KERNEL);
		if (!lcd->tset_table[i]) {
			pr_err("failed to allocate tset\n");
			ret = -ENOMEM;
			goto err_alloc_tset;
		}

		lcd->tset_table[i][0] = SEQ_TSET[0];
		lcd->tset_table[i][1] = TSET_TABLE[i];
	}

	for (i = 0; i < TSET_STATUS_MAX; i++) {
		for (j = 0; j < 2; j++)
			smtd_dbg("0x%02x, ", lcd->tset_table[i][j]);
		smtd_dbg("\n");
	}

	return 0;

err_alloc_tset:
	while (i > 0)
		kfree(lcd->tset_table[--i]);

err_alloc_tset_table:
	return ret;
#endif
}

static int update_brightness(struct lcd_info *lcd, u8 force)
{
	u32 brightness;


	if(lcd->id[0] == 0 && lcd->id[1] == 0 && lcd->id[2] == 0)
		return 0;

	mutex_lock(&lcd->bl_lock);

	brightness = lcd->bd->props.brightness;

	lcd->bl = get_backlight_level_from_brightness(brightness);

	if ((force) || ((lcd->ldi_enable) && (lcd->current_bl != lcd->bl))) {

		s6tnmr7_gamma_ctl(lcd);

		s6tnmr7_aid_parameter_ctl(lcd, force);

		s6tnmr7_set_elvss(lcd, force);

		lcd->current_bl = lcd->bl;

		dev_info(&lcd->ld->dev, "brightness=%d, bl=%d, candela=%d\n", \
			brightness, lcd->bl, candela_table[lcd->bl]);
	}

	mutex_unlock(&lcd->bl_lock);

	return 0;
}


static int s6tnmr7_set_brightness(struct backlight_device *bd)
{
	int ret = 0;
	int brightness = bd->props.brightness;
	struct lcd_info *lcd = bl_get_data(bd);

	/* dev_info(&lcd->ld->dev, "%s: brightness=%d\n", __func__, brightness); */

	if (brightness < MIN_BRIGHTNESS ||
		brightness > bd->props.max_brightness) {
		dev_err(&bd->dev, "lcd brightness should be %d to %d. now %d\n",
			MIN_BRIGHTNESS, lcd->bd->props.max_brightness, brightness);
		return -EINVAL;
	}

	if (lcd->ldi_enable) {
		ret = update_brightness(lcd, 0);
		if (ret < 0) {
			dev_err(&lcd->ld->dev, "err in %s\n", __func__);
			return -EINVAL;
		}
	}

	return ret;
}


static struct lcd_ops s6tnmr7_lcd_ops = {
	.set_power = s6tnmr7_set_power,
	.get_power = s6tnmr7_get_power,
	.check_fb  = s6tnmr7_check_fb,
};

static const struct backlight_ops s6tnmr7_backlight_ops = {
	.get_brightness = s6tnmr7_get_brightness,
	.update_status = s6tnmr7_set_brightness,
};

static int s6tnmr7_probe(struct mipi_dsim_device *dsim)
{
	int ret;
	struct lcd_info *lcd;
	u8 mtp_data[LDI_MTP_LEN] = {0,};
	u8 elvss_data[LDI_ELVSS_LEN] = {0,};

	lcd = kzalloc(sizeof(struct lcd_info), GFP_KERNEL);
	if (!lcd) {
		pr_err("failed to allocate for lcd\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	g_lcd = lcd;

	lcd->ld = lcd_device_register("panel", dsim->dev, lcd, &s6tnmr7_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		pr_err("failed to register lcd device\n");
		ret = PTR_ERR(lcd->ld);
		goto out_free_lcd;
	}

	lcd->bd = backlight_device_register("panel", dsim->dev, lcd, &s6tnmr7_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		pr_err("failed to register backlight device\n");
		ret = PTR_ERR(lcd->bd);
		goto out_free_backlight;
	}

	lcd->dev = dsim->dev;
	lcd->dsim = dsim;
	lcd->bd->props.max_brightness = MAX_BRIGHTNESS;
	lcd->bd->props.brightness = DEFAULT_BRIGHTNESS;
	lcd->power = FB_BLANK_UNBLANK;
	lcd->connected = 1;

	/* dev_set_drvdata(dsim->dev, lcd); */

	mutex_init(&lcd->lock);
	mutex_init(&lcd->bl_lock);

	s6tnmr7_read_id(lcd, lcd->id);
	s6tnmr7_read_mtp(lcd, mtp_data);

	dev_info(&lcd->ld->dev, "ID: %x, %x, %x\n", lcd->id[0], lcd->id[1], lcd->id[2]);

	init_dynamic_aid(lcd);

	ret = init_gamma_table(lcd, mtp_data);
	ret += init_aid_dimming_table(lcd);
	ret += init_elvss_table(lcd, elvss_data);

	if (ret)
		dev_info(&lcd->ld->dev, "gamma table generation is failed\n");

	if (lcd->power == FB_BLANK_POWERDOWN)
		s6tnmr7_power(lcd, FB_BLANK_UNBLANK);
	else
		update_brightness(lcd, 1);

	lcd->ldi_enable = 1;
	dev_info(&lcd->ld->dev, "%s lcd panel driver has been probed.\n", __FILE__);

	s6tnmr7_ldi_init(lcd); /* temp */

	return 0;

out_free_backlight:
	lcd_device_unregister(lcd->ld);
	kfree(lcd);
	return ret;

out_free_lcd:
	kfree(lcd);
	return ret;

err_alloc:
	return ret;
}

static int s6tnmr7_displayon(struct mipi_dsim_device *dsim)
{
	struct lcd_info *lcd = g_lcd;

	s6tnmr7_power(lcd, FB_BLANK_UNBLANK);

	return 0;
}

static int s6tnmr7_suspend(struct mipi_dsim_device *dsim)
{
	struct lcd_info *lcd = g_lcd;

	s6tnmr7_power(lcd, FB_BLANK_POWERDOWN);

	return 0;
}

static int s6tnmr7_resume(struct mipi_dsim_device *dsim)
{
	return 0;
}

struct mipi_dsim_lcd_driver s6tnmr7_mipi_lcd_driver = {
	.probe		= s6tnmr7_probe,
	.displayon	= s6tnmr7_displayon,
	.suspend	= s6tnmr7_suspend,
	.resume		= s6tnmr7_resume,
};
