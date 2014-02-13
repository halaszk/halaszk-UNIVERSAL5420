/* linux/drivers/video/mdnie.c
 *
 * Register interface file for Samsung mDNIe driver
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/mdnie.h>
#include <linux/fb.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/gpio.h>

#include <plat/fb.h>
#include <plat/regs-mdnie.h>
#include <plat/regs-fb-v4.h>
#include <plat/regs-fb.h>

#include <mach/map.h>
#include <plat/gpio-cfg.h>
#include "mdnie.h"

#include "mdnie_table_ch.h"
#include "mdnie_color_tone_s6tnmr7.h"

#define MDNIE_SYSFS_PREFIX		"/sdcard/mdnie/"
#define PANEL_COORDINATE_PATH	"/sys/class/lcd/panel/color_coordinate"

#define SCENARIO_IS_VIDEO(scenario)			(scenario == VIDEO_MODE)
#define SCENARIO_IS_VALID(scenario)			(scenario < SCENARIO_MAX)

#define ACCESSIBILITY_IS_VALID(accessibility)	(accessibility && (accessibility < ACCESSIBILITY_MAX))

#if 0
#define ADDRESS_IS_SCR_WHITE(address)		(address >= S3C_MDNIE_rWHITE_R && address <= S3C_MDNIE_rWHITE_B)
#define ADDRESS_IS_SCR_RGB(address)			(address >= S3C_MDNIE_rRED_R && address <= S3C_MDNIE_rGREEN_B)

#define SCR_RGB_MASK(value)				(value % S3C_MDNIE_rRED_R)
#endif

#define MDNIE_LITE_MAX_SEQ_SIZE 128



static struct class *mdnie_class;
struct mdnie_info *g_mdnie;

static int mdnie_write(unsigned int addr, unsigned int val)
{
//	s3c_mdnie_write(addr, val);

	return 0;
}

static int mdnie_send_sequence(struct mdnie_info *mdnie, const unsigned short *seq)
{
	int ret = 0, i = 0;
	const unsigned short *wbuf = NULL;

	if (IS_ERR_OR_NULL(seq)) {
		dev_err(mdnie->dev, "mdnie sequence is null\n");
		return -EPERM;
	}

	mutex_lock(&mdnie->dev_lock);

	wbuf = seq;

//	mdnie_mask();

	while (wbuf[i] != END_SEQ) {
		ret += mdnie_write(wbuf[i], wbuf[i+1]);
		i += 2;
	}

	mdnie_lite_write(wbuf, i/2);

//	mdnie_unmask();

	mutex_unlock(&mdnie->dev_lock);

	return ret;
}

static struct mdnie_tuning_info *mdnie_request_table(struct mdnie_info *mdnie)
{
	struct mdnie_tuning_info *table = NULL;

	mutex_lock(&mdnie->lock);

	if (ACCESSIBILITY_IS_VALID(mdnie->accessibility)) {
		table = &accessibility_table[mdnie->cabc][mdnie->accessibility];
		goto exit;
	} else if (mdnie->scenario < SCENARIO_MAX) {
		table = &tuning_table[mdnie->cabc][mdnie->mode][mdnie->scenario];
		goto exit;
	}

exit:
	mutex_unlock(&mdnie->lock);

	return table;
}

static void mdnie_update_sequence(struct mdnie_info *mdnie, struct mdnie_tuning_info *table)
{
	unsigned short *wbuf = NULL;

	if (unlikely(mdnie->tuning)) {
		ret = mdnie_request_firmware(mdnie->path, &wbuf, table->name);
		if (ret < 0 && IS_ERR_OR_NULL(wbuf))
			mdnie_send_sequence(mdnie, table->sequence);
		else
			mdnie_send_sequence(mdnie, wbuf);
		kfree(wbuf);
	} else
		mdnie_send_sequence(mdnie, table->sequence);
}

void mdnie_update(struct mdnie_info *mdnie, u8 force)
{
	struct mdnie_tuning_info *table = NULL;

	if (!mdnie->enable && !force) {
		dev_err(mdnie->dev, "mdnie state is off\n");
		return;
	}

	table = mdnie_request_table(mdnie);
	if (!IS_ERR_OR_NULL(table) && !IS_ERR_OR_NULL(table->sequence)) {
		mdnie_update_sequence(mdnie, table);
		dev_info(mdnie->dev, "%s\n", table->name);
	}

	return;
}

static ssize_t mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mdnie->mode);
}

static ssize_t mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);
	unsigned int value = 0;
	int ret;

	ret = kstrtoul(buf, 0, (unsigned long *)&value);
	if (ret < 0)
		return ret;

	dev_info(dev, "%s: value=%d\n", __func__, value);

	if (value >= MODE_MAX) {
		value = STANDARD;
		return -EINVAL;
	}

	mutex_lock(&mdnie->lock);
	mdnie->mode = value;
	mutex_unlock(&mdnie->lock);

	mdnie_update(mdnie, 0);

	return count;
}


static ssize_t scenario_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mdnie->scenario);
}

static ssize_t scenario_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);
	unsigned int value;
	int ret;

	ret = kstrtoul(buf, 0, (unsigned long *)&value);
	if (ret < 0)
		return ret;

	dev_info(dev, "%s: value=%d\n", __func__, value);

	if (!SCENARIO_IS_VALID(value))
		value = UI_MODE;

	mutex_lock(&mdnie->lock);
	mdnie->scenario = value;
	mutex_unlock(&mdnie->lock);

	mdnie_update(mdnie, 0);

	return count;
}

static void mdnie_update_table(struct mdnie_info *mdnie)
{
	struct mdnie_tuning_info *table = NULL;
	unsigned short *wbuf = NULL;
	u8 cabc, mode, scenario;
	int i;
	int ret;

	if (!mdnie->enable) {
		dev_err(mdnie->dev, "mdnie state is off\n");
		return;
	}

	mutex_lock(&mdnie->lock);

	for (cabc = 0; cabc < CABC_MAX; cabc++) {
		for (mode = 0; mode < MODE_MAX; mode++) {
			for (scenario = 0; scenario < SCENARIO_MAX; scenario++) {
				table = &tuning_table[cabc][mode][scenario];
				ret = mdnie_request_firmware(mdnie->path, &wbuf, table->name);

				msleep(100);

				if (ret < 0 && IS_ERR_OR_NULL(wbuf)) {
					mutex_unlock(&mdnie->lock);
					goto exit;
				}
				table->sequence = wbuf;

				dev_dbg(mdnie->dev, "++ %s\n", table->name);
				i = 0;
				while (wbuf[i] != END_SEQ) {
					dev_dbg(mdnie->dev, "%d, 0x%04x, 0x%04x\n",i, table->sequence[i], table->sequence[i+1]);
					i += 2;
					usleep_range(10, 10);
				}
				dev_dbg(mdnie->dev, "-- %s is updated\n", table->name);
			}
		}
	}

	mutex_unlock(&mdnie->lock);

	table = mdnie_request_table(mdnie);
	if (!IS_ERR_OR_NULL(table) && !IS_ERR_OR_NULL(table->sequence)) {
		mdnie_send_sequence(mdnie, table->sequence);
		dev_info(mdnie->dev, "%s\n", table->name);
	}

exit:
	return;
}

static ssize_t tuning_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);
	char *pos = buf;
	struct mdnie_tuning_info *table;
	int ret, i;
	unsigned short *wbuf;

	pos += sprintf(pos, "++ %s: %s\n", __func__, mdnie->path);

	if (!mdnie->tuning) {
		pos += sprintf(pos, "tunning mode is off\n");
		goto exit;
	}

	if (strncmp(mdnie->path, MDNIE_SYSFS_PREFIX, sizeof(MDNIE_SYSFS_PREFIX) - 1)) {
		pos += sprintf(pos, "file path is invalid, %s\n", mdnie->path);
		goto exit;
	}

	table = mdnie_request_table(mdnie);
	if (!IS_ERR_OR_NULL(table)) {
		ret = mdnie_request_firmware(mdnie->path, &wbuf, table->name);
		i = 0;
		while (wbuf[i] != END_SEQ) {
			pos += sprintf(pos, "0x%04x, 0x%04x\n", wbuf[i], wbuf[i+1]);
			i += 2;
		}
		if (!IS_ERR_OR_NULL(wbuf))
			kfree(wbuf);
		pos += sprintf(pos, "%s\n", table->name);
	}

exit:
	pos += sprintf(pos, "-- %s\n", __func__);

	return pos - buf;
}

static ssize_t tuning_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);
	int ret;

	if (sysfs_streq(buf, "0") || sysfs_streq(buf, "1")) {
		ret = kstrtoul(buf, 0, (unsigned long *)&mdnie->tuning);
		if (ret < 0)
			return ret;
		if (!mdnie->tuning)
			memset(mdnie->path, 0, sizeof(mdnie->path));
		dev_info(dev, "%s: %s\n", __func__, mdnie->tuning ? "enable" : "disable");
	} else {
		if (!mdnie->tuning)
			return count;

		if (count > (sizeof(mdnie->path) - sizeof(MDNIE_SYSFS_PREFIX))) {
			dev_err(dev, "file name %s is too long\n", mdnie->path);
			return -ENOMEM;
		}

		memset(mdnie->path, 0, sizeof(mdnie->path));
		snprintf(mdnie->path, sizeof(MDNIE_SYSFS_PREFIX) + count-1, "%s%s", MDNIE_SYSFS_PREFIX, buf);
		dev_info(dev, "%s: %s\n", __func__, mdnie->path);

		mdnie_update_table(mdnie);
	}

	return count;
}

static ssize_t accessibility_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);
	char *pos = buf;
	unsigned short *wbuf;
	int i = 0;

	pos += sprintf(pos, "%d, ", mdnie->accessibility);
	if (mdnie->accessibility == COLOR_BLIND) {
		if (!IS_ERR_OR_NULL(accessibility_table[mdnie->cabc][COLOR_BLIND].sequence)) {
			wbuf = accessibility_table[mdnie->cabc][COLOR_BLIND].sequence;
			while (wbuf[i] != END_SEQ) {
//				if (ADDRESS_IS_SCR_RGB(wbuf[i]))
//					pos += sprintf(pos, "0x%04x, ", wbuf[i+1]);
				i += 2;
			}
		}
	}
	pos += sprintf(pos, "\n");

	return pos - buf;
}

static ssize_t accessibility_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);
	unsigned int value, s[9], cabc, i, len = 0;
	int ret;
	unsigned short *wbuf;
	char str[100] = {0,};

	ret = sscanf(buf, "%d %x %x %x %x %x %x %x %x %x",
		&value, &s[0], &s[1], &s[2], &s[3],
		&s[4], &s[5], &s[6], &s[7], &s[8]);

	dev_info(dev, "%s: value=%d\n", __func__, value);

	if (ret < 0)
		return ret;
	else {
		if (value >= ACCESSIBILITY_MAX)
			value = ACCESSIBILITY_OFF;

		mutex_lock(&mdnie->lock);
		mdnie->accessibility = value;
		if (value == COLOR_BLIND) {
			if (ret != 10) {
				mutex_unlock(&mdnie->lock);
				return -EINVAL;
			}

			for (cabc = 0; cabc < CABC_MAX; cabc++) {
				wbuf = accessibility_table[cabc][COLOR_BLIND].sequence;
				if (IS_ERR_OR_NULL(wbuf))
					continue;
				i = 0;
				while (wbuf[i] != END_SEQ) {
//					if (ADDRESS_IS_SCR_RGB(wbuf[i]))
//						wbuf[i+1] = s[SCR_RGB_MASK(wbuf[i])];
					i += 2;
				}
			}

			i = 0;
			len = sprintf(str + len, "%s: ", __func__);
			while (len < sizeof(str) && i < ARRAY_SIZE(s)) {
				len += sprintf(str + len, "0x%04x, ", s[i]);
				i++;
			}
			dev_info(dev, "%s\n", str);
		}
		mutex_unlock(&mdnie->lock);

		mdnie_update(mdnie, 0);
	}

	return count;
}

static ssize_t bypass_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", mdnie->bypass);
}

static ssize_t bypass_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct mdnie_info *mdnie = dev_get_drvdata(dev);
	struct mdnie_tuning_info *table = NULL;
	unsigned int value;
	int ret;

	ret = kstrtoul(buf, 0, (unsigned long *)&value);
	if (ret)
		return ret;

	dev_info(dev, "%s :: value=%d\n", __func__, value);

	if (ret < 0)
		return ret;
	else {
		if (value >= BYPASS_MAX)
			value = BYPASS_OFF;

		value = (value) ? BYPASS_ON : BYPASS_OFF;

		mutex_lock(&mdnie->lock);
		mdnie->bypass = value;
		mutex_unlock(&mdnie->lock);

		table = &bypass_table[value];
		if (!IS_ERR_OR_NULL(table)) {
			mdnie_update_sequence(mdnie, table);
			dev_info(mdnie->dev, "%s\n", table->name);
		}
	}

	return count;
}

static struct device_attribute mdnie_attributes[] = {
	__ATTR(mode, 0664, mode_show, mode_store),
	__ATTR(scenario, 0664, scenario_show, scenario_store),
	__ATTR(tuning, 0664, tuning_show, tuning_store),
	__ATTR(accessibility, 0664, accessibility_show, accessibility_store),
	__ATTR(bypass, 0664, bypass_show, bypass_store),
	__ATTR_NULL,
};

#if 1
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct mdnie_info *mdnie;
	struct fb_event *evdata = data;
	int fb_blank;

	/* If we aren't interested in this event, skip it immediately ... */
	switch (event) {
	case FB_EVENT_BLANK:
		break;
	default:
		return 0;
	}

	mdnie = container_of(self, struct mdnie_info, fb_notif);
	if (!mdnie)
		return 0;

	fb_blank = *(int *)evdata->data;

	dev_info(mdnie->dev, "fb_blank: %d\n", fb_blank);

	if (fb_blank == FB_BLANK_UNBLANK) {
		pm_runtime_get_sync(mdnie->dev);

		mutex_lock(&mdnie->lock);
		mdnie->enable = TRUE;
		mutex_unlock(&mdnie->lock);

		mdnie_update(mdnie, 1);
	} else if (fb_blank == FB_BLANK_POWERDOWN) {
		mutex_lock(&mdnie->lock);
		mdnie->enable = FALSE;
		mutex_unlock(&mdnie->lock);

		pm_runtime_put_sync(mdnie->dev);
	}

	return 0;
}

static int mdnie_register_fb(struct mdnie_info *mdnie)
{
	memset(&mdnie->fb_notif, 0, sizeof(mdnie->fb_notif));
	mdnie->fb_notif.notifier_call = fb_notifier_callback;
	return fb_register_client(&mdnie->fb_notif);
}
#endif

static int mdnie_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mdnie_info *mdnie;

	mdnie_class = class_create(THIS_MODULE, dev_name(&pdev->dev));
	if (IS_ERR_OR_NULL(mdnie_class)) {
		pr_err("failed to create mdnie class\n");
		ret = -EINVAL;
		goto error0;
	}

	mdnie_class->dev_attrs = mdnie_attributes;

	mdnie = kzalloc(sizeof(struct mdnie_info), GFP_KERNEL);
	if (!mdnie) {
		pr_err("failed to allocate mdnie\n");
		ret = -ENOMEM;
		goto error1;
	}

	mdnie->dev = device_create(mdnie_class, &pdev->dev, 0, &mdnie, "mdnie");
	if (IS_ERR_OR_NULL(mdnie->dev)) {
		pr_err("failed to create mdnie device\n");
		ret = -EINVAL;
		goto error2;
	}

	mdnie->scenario = UI_MODE;
	mdnie->mode = STANDARD;
	mdnie->enable = FALSE;
	mdnie->tuning = FALSE;
	mdnie->accessibility = ACCESSIBILITY_OFF;
	mdnie->cabc = CABC_OFF;
	mdnie->bypass = BYPASS_OFF;

	mutex_init(&mdnie->lock);
	mutex_init(&mdnie->dev_lock);

	platform_set_drvdata(pdev, mdnie);
	dev_set_drvdata(mdnie->dev, mdnie);

#if 1
	mdnie_register_fb(mdnie);

	mdnie->enable = TRUE;
	mdnie_update(mdnie, 1);
#endif

	g_mdnie = mdnie;

	dev_info(mdnie->dev, "registered successfully\n");

	return 0;

error2:
	kfree(mdnie);
error1:
	class_destroy(mdnie_class);
error0:
	return ret;
}

static int mdnie_remove(struct platform_device *pdev)
{
	struct mdnie_info *mdnie = dev_get_drvdata(&pdev->dev);

	class_destroy(mdnie_class);
	kfree(mdnie);

	return 0;
}

static struct platform_driver mdnie_driver = {
	.driver		= {
		.name	= "mdnie",
		.owner	= THIS_MODULE,
	},
	.probe		= mdnie_probe,
	.remove		= mdnie_remove,
};

static int __init mdnie_init(void)
{
	return platform_driver_register(&mdnie_driver);
}
late_initcall(mdnie_init);

static void __exit mdnie_exit(void)
{
	platform_driver_unregister(&mdnie_driver);
}
module_exit(mdnie_exit);

MODULE_DESCRIPTION("mDNIe Driver");
MODULE_LICENSE("GPL");

