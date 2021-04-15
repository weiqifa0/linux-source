#include <dt-bindings/gpio/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>

struct pogopin_dev_data {
    int relay_1_A_en_pin;
    int relay_1_A_en_val;
	
    int relay_2_A_en_pin;
    int relay_2_A_en_val;
	
    int relay_1_B_en_pin;
    int relay_1_B_en_val;
	
    int relay_2_B_en_pin;
    int relay_2_B_en_val;
};

static ssize_t a1_store_en(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct pogopin_dev_data *dev_data = dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        pr_err( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }

    pr_err("%s: en value : %d\n", __func__, (int)value);
    if (value) {
        gpio_direction_output(dev_data->relay_1_A_en_pin, !dev_data->relay_1_A_en_val);
	msleep(100);
        gpio_direction_output(dev_data->relay_1_A_en_pin, dev_data->relay_1_A_en_val);
    } 

    return count;
}

static ssize_t a2_store_en(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct pogopin_dev_data *dev_data = dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        pr_err( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }

    pr_err("%s: en value : %d\n", __func__, (int)value);
    if (value) {
        gpio_direction_output(dev_data->relay_2_A_en_pin, !dev_data->relay_2_A_en_val);
	msleep(100);
        gpio_direction_output(dev_data->relay_2_A_en_pin, dev_data->relay_2_A_en_val);
    } 

    return count;
}

static ssize_t b1_store_en(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct pogopin_dev_data *dev_data = dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        pr_err( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }

    pr_err("%s: en value : %d\n", __func__, (int)value);
    if (value) {
        gpio_direction_output(dev_data->relay_1_B_en_pin, !dev_data->relay_1_B_en_val);
	msleep(100);
        gpio_direction_output(dev_data->relay_1_B_en_pin, dev_data->relay_1_B_en_val);
    } 

    return count;
}

static ssize_t b2_store_en(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct pogopin_dev_data *dev_data = dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;

    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        pr_err( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }

    pr_err("%s: en value : %d\n", __func__, (int)value);
    if (value) {
        gpio_direction_output(dev_data->relay_2_B_en_pin, !dev_data->relay_2_B_en_val);
	msleep(100);
        gpio_direction_output(dev_data->relay_2_B_en_pin, dev_data->relay_2_B_en_val);
    } 

    return count;
}

static DEVICE_ATTR(a1_en, S_IWUSR, NULL, a1_store_en);
static DEVICE_ATTR(a2_en, S_IWUSR, NULL, a2_store_en);
static DEVICE_ATTR(b1_en, S_IWUSR, NULL, b1_store_en);
static DEVICE_ATTR(b2_en, S_IWUSR, NULL, b2_store_en);

static struct device_attribute *pogo_attr_list[] = {
    &dev_attr_a1_en,
    &dev_attr_a2_en,
    &dev_attr_b1_en,
    &dev_attr_b2_en,
};

static int pogo_create_attr(struct device *dev)
{
    int idx, err = 0;
    int num = (int)(sizeof(pogo_attr_list) / sizeof(pogo_attr_list[0]));

    if (dev == NULL) {
        return -EINVAL;
    }

    for (idx = 0; idx < num; idx++) {
        if ((err = device_create_file(dev, pogo_attr_list[idx]))) {
            pr_err("driver_create_file (%s) = %d\n", pogo_attr_list[idx]->attr.name, err);
            break;
        }
    }

    return err;
}

static struct of_device_id pogopin_of_match[] = {
    { .compatible = "pogo-pin" },
    { }
};

MODULE_DEVICE_TABLE(of, pogopin_of_match);

static int pogopin_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    enum of_gpio_flags flags;
    int gpio_1_A,gpio_2_A,gpio_1_B,gpio_2_B;
    int ret;
    int en_value_1_A,en_value_2_A,en_value_1_B,en_value_2_B;
    struct class *dev_class;
    struct device *ctl_dev;
    struct pogopin_dev_data *dev_data;
//    int err;

    pr_err("%s xiaoqienter: %d\n", __func__, __LINE__);
    if (!node)
        return -ENODEV;
	
/*relay-1-A管脚GPIO的申请及设置*/
    gpio_1_A = of_get_named_gpio_flags(node, "relay-1-A-gpio", 0, &flags);
    en_value_1_A = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
    if (!gpio_is_valid(gpio_1_A)) {
        dev_err(&pdev->dev, "invalid en gpio%d\n", gpio_1_A);
    }

    ret = devm_gpio_request(&pdev->dev, gpio_1_A, "relay-1-A-gpio-en");

    if (ret) {
        dev_err(&pdev->dev,
                "failed to request GPIO%d for pogopin-en-gpio\n",
                gpio_1_A);
        return -EINVAL;
    }

    gpio_direction_output(gpio_1_A, en_value_1_A);
	
/*relay-2-A管脚GPIO的申请及设置*/
    gpio_2_A = of_get_named_gpio_flags(node, "relay-2-A-gpio", 0, &flags);
    en_value_2_A = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
    if (!gpio_is_valid(gpio_2_A)) {
        dev_err(&pdev->dev, "invalid en gpio%d\n", gpio_2_A);
    }

    ret = devm_gpio_request(&pdev->dev, gpio_2_A, "relay-2-A-gpio-en");

    if (ret) {
        dev_err(&pdev->dev,
                "failed to request GPIO%d for pogopin-en-gpio\n",
                gpio_2_A);
        return -EINVAL;
    }

    gpio_direction_output(gpio_2_A, en_value_2_A);

/*relay-1-B管脚GPIO的申请及设置*/
    gpio_1_B = of_get_named_gpio_flags(node, "relay-1-B-gpio", 0, &flags);
    en_value_1_B = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
    if (!gpio_is_valid(gpio_1_B)) {
        dev_err(&pdev->dev, "invalid en gpio%d\n", gpio_1_B);
    }

    ret = devm_gpio_request(&pdev->dev, gpio_1_B, "relay-1-B-gpio-en");

    if (ret) {
        dev_err(&pdev->dev,
                "failed to request GPIO%d for pogopin-en-gpio\n",
                gpio_1_B);
        return -EINVAL;
    }

    gpio_direction_output(gpio_1_B, en_value_1_B);

/*relay-2-B管脚GPIO的申请及设置*/
    gpio_2_B = of_get_named_gpio_flags(node, "relay-2-B-gpio", 0, &flags);
    en_value_2_B = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
    if (!gpio_is_valid(gpio_2_B)) {
        dev_err(&pdev->dev, "invalid en gpio%d\n", gpio_2_B);
    }

    ret = devm_gpio_request(&pdev->dev, gpio_2_B, "relay-2-B-gpio-en");

    if (ret) {
        dev_err(&pdev->dev,
                "failed to request GPIO%d for pogopin-en-gpio\n",
                gpio_2_B);
        return -EINVAL;
    }

    pr_err("%s xiaoqi9898:en_value_2_B=%d... %d\n", __func__, en_value_2_B, __LINE__);
    gpio_direction_output(gpio_2_B, en_value_2_B);
	
    dev_data = kzalloc(sizeof(struct pogopin_dev_data), GFP_KERNEL);
    if (dev_data == NULL) {
        pr_err("Failed to malloc pogopin_dev_data\n");
        ret = -ENOMEM;
	goto err_create_dev;

    }

    dev_data->relay_1_A_en_pin = gpio_1_A;
    dev_data->relay_1_A_en_val = en_value_1_A;
	
    dev_data->relay_2_A_en_pin = gpio_2_A;
    dev_data->relay_2_A_en_val = en_value_2_A;
	
    dev_data->relay_1_B_en_pin = gpio_1_B;
    dev_data->relay_1_B_en_val = en_value_2_B;
	
    dev_data->relay_2_B_en_pin = gpio_2_B;
    dev_data->relay_2_B_en_val = en_value_2_B;
	
    dev_info(&pdev->dev, "%s: %d\n", __func__, __LINE__);

    dev_set_drvdata(&pdev->dev, dev_data);
    dev_class = class_create(THIS_MODULE, "pogo-pin");
    ctl_dev = device_create(dev_class, NULL, 0, NULL, "control");
    if (IS_ERR(ctl_dev)) {
        dev_err(ctl_dev, "Failed to create device\n");
        ret = PTR_ERR(ctl_dev);
//        goto err_create_dev;
    }

    dev_set_drvdata(ctl_dev, dev_data);

    if (pogo_create_attr(ctl_dev)) {
        ret = -EINVAL;
//        goto err_create_attr;
    }
	
    return 0;

err_create_dev:
    kfree(dev_data);
    return ret;
}

static int pogopin_remove(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int pogopin_suspend(struct device *dev)
{
    return 0;
}

static int pogopin_resume(struct device *dev)
{
    return 0;
}
#endif

static const struct dev_pm_ops pogopin_pm_ops = {
#ifdef CONFIG_PM_SLEEP
    .suspend = pogopin_suspend,
    .resume = pogopin_resume,
    .poweroff = pogopin_suspend,
    .restore = pogopin_resume,
#endif
};

static struct platform_driver pogopin_driver = {
    .driver		= {
        .name	= "pogo-pin",
        .owner	= THIS_MODULE,
        .pm		= &pogopin_pm_ops,
        .of_match_table	= of_match_ptr(pogopin_of_match),
    },
    .probe		= pogopin_probe,
    .remove		= pogopin_remove,
};

module_platform_driver(pogopin_driver);

MODULE_AUTHOR("xiaoqi@konwin.com>");
MODULE_DESCRIPTION("simple pogopin driver");
MODULE_LICENSE("GPL");
