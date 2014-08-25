#ifndef __MDNIE_H__
#define __MDNIE__

struct platform_mdnie_data {
	unsigned int	display_type;
    unsigned int    support_pwm;
#if defined (CONFIG_S5P_MDNIE_PWM)
    int pwm_out_no;
	int pwm_out_func;
    char *name;
    int *br_table;
    int dft_bl;
#endif
	int (*trigger_set)(struct device *fimd);
	struct device *fimd1_device;
	struct lcd_platform_data	*lcd_pd;
};

#ifdef CONFIG_FB_I80IF
extern int s3c_fb_enable_trigger_by_mdnie(struct device *fimd);
#endif

#endif
