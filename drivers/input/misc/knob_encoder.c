/*
 * linux/drivers/misc/knod_encoder.c
 *
 * knob encoder driver
 *
 * Copyright 2020, Howrd <howrd@21cn.com>
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/async.h>
#include <linux/ioport.h>
#include <linux/wakelock.h>
#include <asm/irq.h>
#include <asm/delay.h>
#include <linux/irq.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/types.h>


struct knob_dev_data {
    int sa_pin;
    unsigned long sa_irq_flags;
    int sa_irq;

    int sb_pin;
    unsigned long sb_irq_flags;
    int sb_irq;

    bool sa_trig;
    bool sb_trig;
    int step_count;

    struct wake_lock work_wake_lock;
    struct input_dev *input;

    struct mutex irq_lock;

    bool irq_enable;

    struct timer_list timer;
};


void knob_enc_set_irq(struct knob_dev_data *dev_data, bool is_enable)
{
    mutex_lock(&dev_data->irq_lock);

    if (is_enable) {
        if (!dev_data->irq_enable) {
            enable_irq(dev_data->sa_irq);
            enable_irq(dev_data->sb_irq);
            dev_data->irq_enable = true;
        }
    } else {
        if (dev_data->irq_enable) {
            disable_irq(dev_data->sa_irq);
            disable_irq(dev_data->sb_irq);
            dev_data->irq_enable = false;
        }
    }

    mutex_unlock(&dev_data->irq_lock);
}

static irqreturn_t knob_encoder_irqhandler(int irq, void *devid)
{
    struct knob_dev_data *dev_data = devid;
    int sa_val, sb_val;

    pr_info("=== Howrd === knob_encoder irq, irq:%d!!!\n", irq);

    //mutex_lock(&dev_data->irq_lock);

    sa_val = gpio_get_value(dev_data->sa_pin);
    sb_val = gpio_get_value(dev_data->sb_pin);

    pr_info("=== sa = %d, sb = %d ===\n", sa_val, sb_val);

    if (!dev_data->sa_trig && !dev_data->sb_trig) {
        if (sa_val == sb_val) {
            dev_data->step_count = 0;
            dev_data->sa_trig = false;
            dev_data->sb_trig = false;
            //mutex_unlock(&dev_data->irq_lock);
            return IRQ_HANDLED;
        }
    }

    //mod_timer(&dev_data->timer, jiffies + msecs_to_jiffies(500));

    if (irq == dev_data->sa_irq) { 	// SA interrupt
        if (dev_data->sb_trig) {
            dev_data->step_count--;
            dev_data->sa_trig = false;
            dev_data->sb_trig = false;

            pr_info("====== UP UP UP ======\n");

            input_report_key(dev_data->input, KEY_UP, 1);
            input_sync(dev_data->input);
            input_report_key(dev_data->input, KEY_UP, 0);
            input_sync(dev_data->input);
        } else {
            dev_data->sa_trig = true;
        }
    } else if (irq == dev_data->sb_irq) { // SB interrupt
        if (dev_data->sa_trig) {
            dev_data->step_count++;
            dev_data->sa_trig = false;
            dev_data->sb_trig = false;

            pr_info("====== DOWN DOWN ======\n");

            input_report_key(dev_data->input, KEY_DOWN, 1);
            input_sync(dev_data->input);
            input_report_key(dev_data->input, KEY_DOWN, 0);
            input_sync(dev_data->input);
        } else {
            dev_data->sb_trig = true;
        }
    }

    //mutex_unlock(&dev_data->irq_lock);
    return IRQ_HANDLED;
}

static void knob_enc_timeout(ulong data)
{
    struct knob_dev_data *dev_data = (struct knob_dev_data *)data;

    pr_info("=== Howrd === timeout!\n");
    if (dev_data->sa_trig || dev_data->sb_trig) {
        dev_data->step_count = 0;
        dev_data->sa_trig = false;
        dev_data->sb_trig = false;
    }
}

static struct of_device_id knob_encoder_of_match[] = {
    { .compatible = "mediatek,knob_encoder" },
    { }
};

MODULE_DEVICE_TABLE(of, knob_encoder_of_match);


static int knob_encoder_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    enum of_gpio_flags irq_flags;
    struct input_dev *input_device;
    int ret;
    int error;
    struct knob_dev_data *dev_data;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;

    pr_info("=== Howrd === %s enter!\n", __func__);
    if (!node)
        return -ENODEV;

    dev_data = kzalloc(sizeof(struct knob_dev_data), GFP_KERNEL);
    if (dev_data == NULL) {
        pr_err("Failed to malloc knob_dev_data\n");
        return -ENOMEM;
    }
    pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("Cannot find knbo pinctrl!\n");
		return ret;
	}else{
		pins_default = pinctrl_lookup_state(pinctrl, "default");
		if (IS_ERR(pins_default)) {
			ret = PTR_ERR(pins_default);
			pr_err("Cannot find knbo pinctrl default!\n");
		}else{
			pinctrl_select_state(pinctrl, pins_default);
		}
	}
    dev_data->sa_pin = of_get_named_gpio_flags(node, "signal-a-gpio", 0, &irq_flags);
    dev_data->sa_irq_flags = irq_flags;
    dev_data->sb_pin = of_get_named_gpio_flags(node, "signal-b-gpio", 0, &irq_flags);
    dev_data->sb_irq_flags = irq_flags;
    mutex_init(&dev_data->irq_lock);

    /* setup timer */
    init_timer(&dev_data->timer);
    dev_data->timer.data = (ulong)dev_data;
    dev_data->timer.function = knob_enc_timeout;

    wake_lock_init(&dev_data->work_wake_lock, WAKE_LOCK_SUSPEND, "knob_enc");

    input_device = input_allocate_device();
    if (!input_device) {
        ret = -ENOMEM;
        goto error_alloc_dev;
    }

    dev_data->input = input_device;
    input_device->name = "knob-keys";
    input_device->dev.parent = &pdev->dev;
    input_set_drvdata(input_device, dev_data);

    __set_bit(EV_KEY, input_device->evbit);
    __set_bit(EV_SYN, input_device->evbit);
    input_set_capability(input_device, EV_KEY, KEY_UP);
    input_set_capability(input_device, EV_KEY, KEY_DOWN);

    ret = input_register_device(input_device);
    if (ret) {
        goto err_input_register;
    }

    if (gpio_is_valid(dev_data->sa_pin)) {
        ret = gpio_request(dev_data->sa_pin, "knob-enc sa irq");
        if (ret < 0) {
            dev_err(&pdev->dev, "%s: 111 failed to set gpio int.\n", __func__);
            goto err_req_a_pin;
        }

        gpio_direction_input(dev_data->sa_pin);

        dev_data->sa_irq = gpio_to_irq(dev_data->sa_pin);
        if (dev_data->sa_irq < 0) {
            error = dev_data->sa_irq;
            dev_err(&pdev->dev, "Unable to get irq number for GPIO %d, error %d\n",
                    dev_data->sa_pin, error);
            goto err_req_a_irq;
        }

        error = devm_request_threaded_irq(&pdev->dev, dev_data->sa_irq, NULL,     \
                                          knob_encoder_irqhandler,                \
                                          dev_data->sa_irq_flags | IRQF_ONESHOT,  \
                                          dev_name(&pdev->dev),                   \
                                          dev_data);
        if (error < 0) {
            dev_err(&pdev->dev, "request irq(%d) failed:%d\n",
                    dev_data->sa_irq, error);
            goto err_req_a_irq;
        }

        mutex_lock(&dev_data->irq_lock);
        disable_irq(dev_data->sa_irq);
        mutex_unlock(&dev_data->irq_lock);
    }

    if (gpio_is_valid(dev_data->sb_pin)) {
        ret = gpio_request(dev_data->sb_pin, "knob-enc sb irq");
        if (ret < 0) {
            dev_err(&pdev->dev, "%s: 222 failed to set gpio int.\n", __func__);
            goto err_req_b_pin;
        }

        gpio_direction_input(dev_data->sb_pin);

        dev_data->sb_irq = gpio_to_irq(dev_data->sb_pin);
        if (dev_data->sb_irq < 0) {
            error = dev_data->sb_irq;
            dev_err(&pdev->dev, "Unable to get irq number for GPIO %d, error %d\n",
                    dev_data->sb_pin, error);
            goto err_req_b_irq;
        }

        error = devm_request_threaded_irq(&pdev->dev, dev_data->sb_irq, NULL,     \
                                          knob_encoder_irqhandler,                \
                                          dev_data->sb_irq_flags | IRQF_ONESHOT,  \
                                          dev_name(&pdev->dev),                   \
                                          dev_data);
        if (error < 0) {
            dev_err(&pdev->dev, "request irq(%d) failed:%d\n",
                    dev_data->sb_irq, error);
            goto err_req_b_irq;
        }

        mutex_lock(&dev_data->irq_lock);
        disable_irq(dev_data->sb_irq);
        mutex_unlock(&dev_data->irq_lock);
    }

    pr_info("=== Howrd === irqa=%d, irqb=%d\n", dev_data->sa_irq, dev_data->sb_irq);

    dev_set_drvdata(&pdev->dev, dev_data);

    knob_enc_set_irq(dev_data, true);
    return 0;

err_req_b_irq:
    if (gpio_is_valid(dev_data->sb_pin)) {
        gpio_free(dev_data->sb_pin);
    }

err_req_b_pin:
    free_irq(dev_data->sa_irq, dev_data);

err_req_a_irq:
    if (gpio_is_valid(dev_data->sa_pin)) {
        gpio_free(dev_data->sa_pin);
    }

err_req_a_pin:
    input_unregister_device(input_device);

err_input_register:
    input_free_device(input_device);

error_alloc_dev:
    wake_lock_destroy(&dev_data->work_wake_lock);
    del_timer(&dev_data->timer);
    mutex_destroy(&dev_data->irq_lock);
    kfree(dev_data);

    return ret;
}

static int knob_encoder_remove(struct platform_device *pdev)
{
    //pr_info("func: %s\n", __func__);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int knob_encoder_suspend(struct device *dev)
{
    struct knob_dev_data *dev_data = dev_get_drvdata(dev);
    pr_info("func: %s\n", __func__);
    if (device_may_wakeup(dev)) {
        enable_irq_wake(dev_data->sa_irq);
        enable_irq_wake(dev_data->sb_irq);
    }
    return 0;
}

static int knob_encoder_resume(struct device *dev)
{
    struct knob_dev_data *dev_data = dev_get_drvdata(dev);
    pr_info("func: %s\n", __func__);
    if (device_may_wakeup(dev)) {
        disable_irq_wake(dev_data->sa_irq);
        disable_irq_wake(dev_data->sb_irq);
    }
    return 0;
}
#endif

static const struct dev_pm_ops knob_encoder_pm_ops = {
#ifdef CONFIG_PM_SLEEP
    .suspend = knob_encoder_suspend,
    .resume = knob_encoder_resume,
    .poweroff = knob_encoder_suspend,
    .restore = knob_encoder_resume,
#endif
};

static struct platform_driver knob_encoder_driver = {
    .driver		= {
        .name		= "knob_encoder",
        .owner		= THIS_MODULE,
        .pm		= &knob_encoder_pm_ops,
        .of_match_table	= of_match_ptr(knob_encoder_of_match),
    },
    .probe		= knob_encoder_probe,
    .remove		= knob_encoder_remove,
};

module_platform_driver(knob_encoder_driver);

MODULE_AUTHOR("Howrd <howrd@21cn.com>");
MODULE_DESCRIPTION("knob encoder Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:knob_encoder");