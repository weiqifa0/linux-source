#include <dt-bindings/gpio/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>

struct gpio_dev_data {
    int en_pin;
    int en_val;
    int gpio_val;
};

static struct of_device_id gpio_of_match[] = {
    { .compatible = "gpio,normal-gpio" },
    { }
};

MODULE_DEVICE_TABLE(of, gpio_of_match);


static ssize_t gpio_store_en(struct device *dev,
                              struct device_attribute *attr,
                              const char *buf, size_t count)
{
    struct gpio_dev_data *dev_data = dev_get_drvdata(dev);
    unsigned long value = 0;
    int ret;

    /*将echo进来的buf转换成整型*/
    ret = kstrtoul(buf, 16, &value);
    if (ret < 0) {
        printk( "%s:kstrtoul failed, ret=%d\n", __func__, ret);
        return ret;
    }

    printk("%s: en value : %d\n", __func__, (int)value);
    if (value) {
        gpio_direction_output(dev_data->en_pin, dev_data->en_val);
        dev_data->gpio_val = 1;
    } else {
        gpio_direction_output(dev_data->en_pin, !dev_data->en_val);
        dev_data->gpio_val = 0;
    }

    return count;
}

static  char mybuf[10]="123";
 /*cat命令时,将会调用该函数*/
static ssize_t gpio_show_en(struct device *dev,
                  struct device_attribute *attr, char *buf)       
{
    struct gpio_dev_data *dev_data = dev_get_drvdata(dev);
    snprintf(mybuf,sizeof(mybuf),"%d",dev_data->gpio_val);
    return sprintf(buf, "%s\n", mybuf);
}

static DEVICE_ATTR(gpio_en, S_IWUSR, gpio_show_en, gpio_store_en);


static int gpio_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    enum of_gpio_flags flags;
    int gpio;
    int ret;
    int en_value;
    struct class *dev_class;
    struct device *ctl_dev;
    struct gpio_dev_data *dev_data;
    int err;
    const char *class_name;

    printk("%s enter: %d\n", __func__, __LINE__);
    if (!node)
        return -ENODEV;

    /*获取gpio口*/
    gpio = of_get_named_gpio_flags(node, "en-gpio", 0, &flags);
    /*读取dts里面gpio口的默认电平状态*/
    en_value = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
    if (!gpio_is_valid(gpio)) {
        dev_err(&pdev->dev, "invalid en gpio%d\n", gpio);
    }

    printk("%s enter: LINE-%d,GPIO-%d\n", __func__, __LINE__,gpio);
    /*把gpio设置为onoff名字*/
    ret = devm_gpio_request(&pdev->dev, gpio, "onoff");
    if (ret) {
        dev_err(&pdev->dev,
                "line-%d,failed to request GPIO:%d for \n",
                __LINE__,gpio);
        return -EINVAL;
    }

    /*设置probe的默认电平*/
    gpio_direction_output(gpio, en_value);
    dev_data = kzalloc(sizeof(struct gpio_dev_data), GFP_KERNEL);
    if (dev_data == NULL) {
        printk("Failed to malloc gpio_dev_data\n");
        return -ENOMEM;
    }

    dev_data->gpio_val = en_value;
    /*创建class节点*/
    err = of_property_read_string(node, "cname", &class_name);
    if(err){
        printk("%s get class name error %d\n",__FUNCTION__,__LINE__);
        return -EINVAL;
    }
    dev_class = class_create(THIS_MODULE, class_name);
    ctl_dev = device_create(dev_class, NULL, 0, NULL, "onoff");
    if (IS_ERR(ctl_dev)) {
        dev_err(ctl_dev, "Failed to create device\n");
        ret = PTR_ERR(ctl_dev);
        goto err_create_dev;
    }

    err = device_create_file(ctl_dev, &dev_attr_gpio_en);
    if (err){
        printk("driver_create_file = %d\n", err);
    }

    /*获取的gpio赋值全局变量*/
    dev_data->en_pin = gpio;
    dev_data->en_val = en_value;

    dev_info(ctl_dev, "%s: %d\n", __func__, __LINE__);
    dev_info(&pdev->dev, "%s: %d\n", __func__, __LINE__);

    dev_set_drvdata(ctl_dev, dev_data);
    dev_set_drvdata(&pdev->dev, dev_data);

    return 0;

err_create_dev:
    kfree(dev_data);
    return ret;
}

static int gpio_remove(struct platform_device *pdev)
{
    printk("%s: %d\n", __func__, __LINE__);
    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_suspend(struct device *dev)
{
    struct gpio_dev_data *dev_data = dev_get_drvdata(dev);
    dev_info(dev, "%s: %d\n", __func__, __LINE__);

    gpio_direction_output(dev_data->en_pin, !dev_data->en_val);
    return 0;
}

static int gpio_resume(struct device *dev)
{
    printk("%s: %d\n", __func__, __LINE__);
    return 0;
}
#endif

static const struct dev_pm_ops gpio_pm_ops = {
#ifdef CONFIG_PM_SLEEP
    .suspend = gpio_suspend,
    .resume = gpio_resume,
    .poweroff = gpio_suspend,
    .restore = gpio_resume,
#endif
};

static struct platform_driver gpio_driver = {
    .driver		= {
        .name	= "gpio-en",
        .owner	= THIS_MODULE,
        .pm		= &gpio_pm_ops,
        .of_match_table	= of_match_ptr(gpio_of_match),
    },
    .probe		= gpio_probe,
    .remove		= gpio_remove,
};

module_platform_driver(gpio_driver);

MODULE_AUTHOR("Weiqifa <329410527@qq.com>");
MODULE_DESCRIPTION("simple gpio driver");
MODULE_LICENSE("GPL");
