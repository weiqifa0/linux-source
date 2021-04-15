/*
 *
 * Copyright 2020
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
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
#include <asm/irq.h>
#include <asm/delay.h>
#include <linux/irq.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/types.h>
#include <linux/input.h>

#define STATE_ON	1
#define STATE_OFF	0


struct cam_ctl_dev_data {
    int cam_pin;
    unsigned long cam_irq_flags;
    int cam_irq;
    bool irq_enable;
    int cam_state;
    struct input_dev *input;
};

static irqreturn_t cam_ctl_irqhandler(int irq, void *devid)
{
    struct cam_ctl_dev_data *dev_data = devid;
    int cam_irq_vall;

    cam_irq_vall = gpio_get_value(dev_data->cam_pin);
	pr_info("cam_ctl=== cam_irq_vall = %d,===\n",cam_irq_vall);

	if (!cam_irq_vall) {
		dev_data->cam_state = 1;
	}else{
		dev_data->cam_state = 0;
	}

	input_report_key(dev_data->input, KEY_F13, 1);
	input_sync(dev_data->input);
	input_report_key(dev_data->input, KEY_F13, 0);
	input_sync(dev_data->input);

    return IRQ_HANDLED;
}

static ssize_t cam_show_state(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct cam_ctl_dev_data *dev_data = dev_get_drvdata(dev);
    return sprintf(buf, "%d\n",  dev_data->cam_state);
}

static ssize_t cam_store_state(struct device *dev,
                                struct device_attribute *attr,
                                const char *buf, size_t count)
{
    struct cam_ctl_dev_data *dev_data = dev_get_drvdata(dev);
	int buf_value = 0;
	sscanf(buf, "%d", &buf_value);

	if ( dev_data->cam_state != buf_value )
	{
		printk("set state pin:%d\n",buf_value);
		if ( buf_value == STATE_ON )
			dev_data->cam_state = STATE_ON;
		else if ( buf_value == STATE_OFF )
			dev_data->cam_state = STATE_OFF;
		else
			pr_err(" invaild state\n");
	}
	return count;
}


static DEVICE_ATTR(cam_swt_state,  S_IWUSR | S_IRUGO, cam_show_state,cam_store_state);

static struct device_attribute *cam_ctl_attr_list[] = {
    &dev_attr_cam_swt_state,
};

static int cam_ctl_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(cam_ctl_attr_list) / sizeof(cam_ctl_attr_list[0]));

    if (dev == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((err = device_create_file(dev, cam_ctl_attr_list[idx]))) {
            pr_err("driver_create_file (%s) = %d\n", cam_ctl_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



static struct of_device_id cam_ctl_of_match[] = {
    { .compatible = "mediatek,cam_ctl" },
    { }
};

MODULE_DEVICE_TABLE(of, cam_ctl_of_match);

static int cam_ctl_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    enum of_gpio_flags irq_flags;
    int ret;
    int error;
    struct cam_ctl_dev_data *dev_data;
    struct pinctrl *pinctrl;
    struct pinctrl_state *pins_default;
    struct input_dev *input_device;

    pr_info("===   === %s enter!\n", __func__);
    if (!node)
        return -ENODEV;

    dev_data = kzalloc(sizeof(struct cam_ctl_dev_data), GFP_KERNEL);
    if (dev_data == NULL) {
        pr_err("Failed to malloc cam_ctl_dev_data\n");
        return -ENOMEM;
    }
    pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("Cannot find  pinctrl!\n");
		return ret;
	}else{
		pins_default = pinctrl_lookup_state(pinctrl, "default");
		if (IS_ERR(pins_default)) {
			ret = PTR_ERR(pins_default);
			pr_err("Cannot find  pinctrl default!\n");
		}else{
			pinctrl_select_state(pinctrl, pins_default);
		}
	}
    dev_data->cam_pin = of_get_named_gpio_flags(node, "cam-ctl-gpio", 0, &irq_flags);
    dev_data->cam_irq_flags = irq_flags;

    if (gpio_is_valid(dev_data->cam_pin)) {
        ret = gpio_request(dev_data->cam_pin, "cam_ctl sa irq");
        if (ret < 0) {
            dev_err(&pdev->dev, "%s: 111 failed to set gpio int.\n", __func__);
        }

        gpio_direction_input(dev_data->cam_pin);

        dev_data->cam_irq = gpio_to_irq(dev_data->cam_pin);
        if (dev_data->cam_irq < 0) {
            error = dev_data->cam_irq;
            dev_err(&pdev->dev, "Unable to get irq number for GPIO %d, error %d\n",
                    dev_data->cam_pin, error);
        }

        error = devm_request_threaded_irq(&pdev->dev, dev_data->cam_irq, NULL,     \
                                          cam_ctl_irqhandler,                \
                                          dev_data->cam_irq_flags | IRQF_ONESHOT,  \
                                          dev_name(&pdev->dev),                   \
                                          dev_data);
        if (error < 0) {
            dev_err(&pdev->dev, "request irq(%d) failed:%d\n",
                    dev_data->cam_irq, error);
        }

       // disable_irq(dev_data->cam_irq);
    }

      input_device = input_allocate_device();
	  if (!input_device) {
		  ret = -ENOMEM;
		  goto error_alloc_dev;
	  }

	  dev_data->input = input_device;
	  input_device->name = "cam-det";
	  input_device->dev.parent = &pdev->dev;
	  input_set_drvdata(input_device, dev_data);

	  __set_bit(EV_KEY, input_device->evbit);
	  __set_bit(EV_SYN, input_device->evbit);
	  input_set_capability(input_device, EV_KEY, KEY_F13);

	  ret = input_register_device(input_device);
	  if (ret) {
		  goto err_input_register;
	  }

    pr_info("===  irq=%d, \n", dev_data->cam_irq);
    cam_ctl_create_attr(&pdev->dev);
    dev_set_drvdata(&pdev->dev, dev_data);

    return 0;

  err_input_register:
     input_unregister_device(input_device);
     input_free_device(input_device); 
  error_alloc_dev:
     return ret;

}

static int cam_ctl_remove(struct platform_device *pdev)
{
    //pr_info("func: %s\n", __func__);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int cam_ctl_suspend(struct device *dev)
{
 //   struct cam_ctl_dev_data *dev_data = dev_get_drvdata(dev);
    pr_info("func: %s\n", __func__);

    return 0;
}

static int cam_ctl_resume(struct device *dev)
{
   // struct cam_ctl_dev_data *dev_data = dev_get_drvdata(dev);
    pr_info("func: %s\n", __func__);

    return 0;
}
#endif

static const struct dev_pm_ops cam_ctl_pm_ops = {
#ifdef CONFIG_PM_SLEEP
    .suspend = cam_ctl_suspend,
    .resume = cam_ctl_resume,
    .poweroff = cam_ctl_suspend,
    .restore = cam_ctl_resume,
#endif
};

static struct platform_driver cam_ctl_driver = {
    .driver		= {
        .name		= "cam_ctl",
        .owner		= THIS_MODULE,
        .pm		= &cam_ctl_pm_ops,
        .of_match_table	= of_match_ptr(cam_ctl_of_match),
    },
    .probe		= cam_ctl_probe,
    .remove		= cam_ctl_remove,
};

module_platform_driver(cam_ctl_driver);

MODULE_AUTHOR(" know");
MODULE_DESCRIPTION("cam_ctl  Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cam_ctl_");
