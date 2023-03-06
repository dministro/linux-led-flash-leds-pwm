// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Jozsef Horvath <info@ministro.hu>
 *
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <media/v4l2-flash-led-class.h>

#define FLASH_TIMEOUT_MIN			100000	/* us */
#define FLASH_TIMEOUT_MAX			1000000
#define FLASH_TIMEOUT_STEP			50000

#define FLASH_INTENSITY_MIN			0
#define FLASH_INTENSITY_MAX			10000
#define FLASH_INTENSITY_STEP			1

#define TORCH_INTENSITY_MIN			FLASH_INTENSITY_MIN
#define TORCH_INTENSITY_MAX			FLASH_INTENSITY_MAX
#define TORCH_INTENSITY_STEP			FLASH_INTENSITY_STEP

#define LEDS_FLASH_PWM_NAME			"leds-flash-pwm"

struct leds_flash_pwm {
	/* platform device data */
	struct platform_device *pdev;
	/* secures access to the device */
	struct mutex lock;
	/* corresponding LED Flash class device */
	struct led_classdev_flash fled_cdev;
	/* V4L2 Flash device */
	struct v4l2_flash *v4l2_flash;
	struct v4l2_subdev v4l2_subdev;
	/* Enable pin */
	struct gpio_desc *gpio_fl_en;
	struct pwm_device *pwm;
	struct pwm_state pwmstate;
	unsigned int active_low;
	/* device mode */
	bool movie_mode;
	/* brightness cache */
	unsigned int torch_brightness;
	bool has_external_strobe;
};

static struct leds_flash_pwm *fled_cdev_to_led(
	struct led_classdev_flash *fled_cdev)
{
	return container_of(fled_cdev, struct leds_flash_pwm, fled_cdev);
}

static struct led_classdev_flash *led_cdev_to_fled_cdev(
	struct led_classdev *led_cdev)
{
	return container_of(led_cdev, struct led_classdev_flash, led_cdev);
}

static int leds_flash_pwm_led_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = led_cdev_to_fled_cdev(led_cdev);
	struct leds_flash_pwm *led = fled_cdev_to_led(fled_cdev);

	mutex_lock(&led->lock);

	if (brightness == 0) {
		gpiod_direction_output(led->gpio_fl_en, 0);
		led->movie_mode = false;
	} else {
		if (!led->movie_mode) {
			/* TODO */
			led->movie_mode = true;
		}

		/* TODO */
	}

	mutex_unlock(&led->lock);

	return 0;
}

static int leds_flash_pwm_led_flash_strobe_set(struct led_classdev_flash *fled_cdev,
						bool state)
{
	struct leds_flash_pwm *led = fled_cdev_to_led(fled_cdev);
	struct led_classdev *led_cdev = &fled_cdev->led_cdev;
	struct led_flash_setting __attribute__((unused)) *timeout = &fled_cdev->timeout;

	mutex_lock(&led->lock);

	if (state) {
		/* TODO */
		//leds_flash_pwm_set_flash_safety_timer(led, timeout->val);
		gpiod_direction_output(led->gpio_fl_en, 1);
	} else {
		gpiod_direction_output(led->gpio_fl_en, 0);
	}

	/* TODO */
	led_cdev->brightness = 0;
	led->movie_mode = false;

	mutex_unlock(&led->lock);

	return 0;
}

static int leds_flash_pwm_led_flash_timeout_set(struct led_classdev_flash *fled_cdev,
						u32 timeout)
{
	/* TODO */

	return 0;
}

static void leds_flash_pwm_init_flash_timeout(struct leds_flash_pwm *led)
{
	struct led_classdev_flash *fled_cdev = &led->fled_cdev;
	struct led_flash_setting *timeout;

	/* Init flash timeout setting */
	timeout = &fled_cdev->timeout;
	timeout->min = FLASH_TIMEOUT_MIN;
	timeout->max = FLASH_TIMEOUT_MAX;
	timeout->step = FLASH_TIMEOUT_STEP;
	timeout->val = FLASH_TIMEOUT_MIN;
}

static enum led_brightness leds_flash_pwm_intensity_to_brightness(
	struct v4l2_flash *v4l2_flash,
	s32 intensity)
{
	struct led_classdev_flash *fled_cdev = v4l2_flash->fled_cdev;
	struct leds_flash_pwm *led = fled_cdev_to_led(fled_cdev);
	int i;

	/* TODO */
	/*for (i = AAT1290_MM_CURRENT_SCALE_SIZE - 1; i >= 0; --i)
		if (intensity >= led->mm_current_scale[i])
			return i + 1;*/

	return 1;
}

static s32 leds_flash_pwm_brightness_to_intensity(struct v4l2_flash *v4l2_flash,
	enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = v4l2_flash->fled_cdev;
	struct leds_flash_pwm *led = fled_cdev_to_led(fled_cdev);

	return 0;
}

static int leds_flash_pwm_led_external_strobe_set(struct v4l2_flash *v4l2_flash,
	bool enable)
{
	struct leds_flash_pwm *led = fled_cdev_to_led(v4l2_flash->fled_cdev);
	struct led_classdev_flash *fled_cdev = v4l2_flash->fled_cdev;
	struct led_classdev *led_cdev = &fled_cdev->led_cdev;
	struct pinctrl *pinctrl;

	gpiod_direction_output(led->gpio_fl_en, 0);

	led->movie_mode = false;
	led_cdev->brightness = 0;

	pinctrl = devm_pinctrl_get_select(&led->pdev->dev,
		enable ? "isp" : "host");
	if (IS_ERR(pinctrl)) {
		dev_warn(&led->pdev->dev, "Unable to switch strobe source.\n");
		return PTR_ERR(pinctrl);
	}

	return 0;
}

static void leds_flash_pwm_init_v4l2_flash_config(struct leds_flash_pwm *led,
	struct v4l2_flash_config *v4l2_sd_cfg)
{
	struct led_classdev *led_cdev = &led->fled_cdev.led_cdev;
	struct led_flash_setting *intensity;

	strlcpy(v4l2_sd_cfg->dev_name, led_cdev->dev->kobj.name,
		sizeof(v4l2_sd_cfg->dev_name));

	intensity = &v4l2_sd_cfg->intensity;
	intensity->min = FLASH_INTENSITY_MIN;
	intensity->max = FLASH_INTENSITY_MAX;
	intensity->step = FLASH_INTENSITY_STEP;
	intensity->val = intensity->max;

	v4l2_sd_cfg->has_external_strobe = led->has_external_strobe;
}

static int leds_flash_pwm_probe_dt(struct leds_flash_pwm *priv,
	struct device_node **sub_node)
{
	struct device *dev = &priv->pdev->dev;
	struct pinctrl *pinctrl;
	int ret;

	priv->gpio_fl_en = devm_gpiod_get_optional(dev, "flen", GPIOD_OUT_LOW);
	if (!priv->gpio_fl_en)
		dev_err(dev, "enable gpio is not assigned!\n");

	ret = PTR_ERR_OR_ZERO(priv->gpio_fl_en);
	if (ret) {
		dev_err(dev, "Error %d while getting enable gpio\n", ret);
		return ret;
	}

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		priv->has_external_strobe = false;
		dev_info(dev,
			 "No support for external strobe detected.\n");
	} else {
		priv->has_external_strobe = true;
	}

	*sub_node = of_get_next_available_child(dev->of_node, NULL);
	if (!*sub_node) {
		dev_err(dev, "No DT child node found for connected LED.\n");
		return -EINVAL;
	}

	return ret;
}

static const struct v4l2_flash_ops v4l2_flash_ops = {
	.external_strobe_set = leds_flash_pwm_led_external_strobe_set,
	.intensity_to_led_brightness = leds_flash_pwm_intensity_to_brightness,
	.led_brightness_to_intensity = leds_flash_pwm_brightness_to_intensity,
};

static const struct led_flash_ops flash_ops = {
	.strobe_set = leds_flash_pwm_led_flash_strobe_set,
	.timeout_set = leds_flash_pwm_led_flash_timeout_set,
};

static int leds_flash_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct leds_flash_pwm *led;
	struct led_classdev *led_cdev;
	struct led_classdev_flash *fled_cdev;
	struct v4l2_flash_config v4l2_sd_cfg = {};
	struct device_node *sub_node;
	int ret;

	led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
	if (!led)
		return -ENOMEM;

	led->pdev = pdev;
	
	platform_set_drvdata(pdev, led);

	fled_cdev = &led->fled_cdev;
	fled_cdev->ops = &flash_ops;
	led_cdev = &fled_cdev->led_cdev;

	ret = leds_flash_pwm_probe_dt(led, &sub_node);
	if (ret)
		return ret;

	mutex_init(&led->lock);

	led_cdev->brightness_set_blocking = leds_flash_pwm_led_brightness_set;
	led_cdev->max_brightness = FLASH_INTENSITY_MAX;
	led_cdev->flags |= LED_DEV_CAP_FLASH;
	led_cdev->name = of_get_property(sub_node, "label", NULL) ? :
						sub_node->name;

	leds_flash_pwm_init_flash_timeout(led);
	
	/* Register LED Flash class device */
	ret = led_classdev_flash_register(&pdev->dev, fled_cdev);
	if (ret < 0)
		goto err_flash_register;

	leds_flash_pwm_init_v4l2_flash_config(led, &v4l2_sd_cfg);

	/* Create V4L2 Flash subdev. */
	dev_err(dev, "leds_flash_pwm_probe:subdev_handle:%px", of_fwnode_handle(sub_node));
	led->v4l2_flash = v4l2_flash_init(dev, of_fwnode_handle(sub_node),
		fled_cdev, &v4l2_flash_ops, &v4l2_sd_cfg);
	if (IS_ERR(led->v4l2_flash)) {
		ret = PTR_ERR(led->v4l2_flash);
		goto error_v4l2_flash_init;
	}
	dev_err(dev, "ctrl_handler=%px\n", led->v4l2_flash->hdl);

	return 0;

error_v4l2_flash_init:
	led_classdev_flash_unregister(fled_cdev);
err_flash_register:
	mutex_destroy(&led->lock);

	return ret;
}

static int leds_flash_pwm_remove(struct platform_device *pdev)
{
	struct leds_flash_pwm *led = platform_get_drvdata(pdev);

	v4l2_flash_release(led->v4l2_flash);
	led_classdev_flash_unregister(&led->fled_cdev);

	mutex_destroy(&led->lock);

	return 0;
}

static const struct of_device_id of_leds_flash_pwm_match[] = {
	{ .compatible = LEDS_FLASH_PWM_NAME, },
	{},
};
MODULE_DEVICE_TABLE(of, of_leds_flash_pwm_match);

static struct platform_driver leds_flash_pwm_driver = {
	.probe		= leds_flash_pwm_probe,
	.remove		= leds_flash_pwm_remove,
	.driver		= {
		.name	= "leds_flash_pwm",
		.of_match_table = of_leds_flash_pwm_match,
	},
};

module_platform_driver(leds_flash_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jozsef Horvath <info@ministro.hu>");
MODULE_DESCRIPTION("PWM led flash driver");
