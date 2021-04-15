#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/workqueue.h>
#include <asm/mach-types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
 
#include "timed_output.h"

static int gpio, en_value;

#define MAX_TIMEOUT 10000/* 10s */  //最长可打开10s
 
static struct vibrator {
	struct wake_lock wklock; //wake_lock 防止震动过程中系统休眠,线程不释放此设备,造成不必要错误
	struct hrtimer timer; //高精度定时器
	struct mutex lock; //互斥锁,防止多线程同时访问这个设备.
	struct work_struct work; //设备操作队列,用于一次操作完成和下一次开始同步用 (三星这么用的，具体为什么不直接用回调函数，我也不懂,还望大神们私信给个说明 感激不尽)
} vibdata;
 
static void vibrator_off(void)
{
	gpio_direction_output(gpio, !en_value);       
	wake_unlock(&vibdata.wklock); //震动关闭就可以释放 wake_lock锁        
}
void motor_enable(struct timed_output_dev *sdev,int value)
{
	mutex_lock(&vibdata.lock); //关键代码段,同一时间只允许一个线程执行

	/* cancelprevious timer and set GPIO according to value */
	hrtimer_cancel(&vibdata.timer); //当先前定时器完成后 关闭这个定时器
	cancel_work_sync(&vibdata.work); //当上次震动完成后 关闭这次动作
	if(value)
	{
		wake_lock(&vibdata.wklock); //开始震动打开wake lock锁不允许休眠
		gpio_direction_output(gpio, en_value);

		if(value > 0)
		{
			if(value > MAX_TIMEOUT)
				value= MAX_TIMEOUT;
			hrtimer_start(&vibdata.timer,
			         ktime_set(value / 1000, (value % 1000) * 1000000),
			         HRTIMER_MODE_REL);
		}
	}
	else
		vibrator_off();

	mutex_unlock(&vibdata.lock);
}
int get_time(struct timed_output_dev *sdev)
{
	if(hrtimer_active(&vibdata.timer))
	{
		ktime_t r = hrtimer_get_remaining(&vibdata.timer); //读取剩余时间按并返回
		return ktime_to_ms(r);
	}

	return 0;
}
struct timed_output_dev motot_driver = {
	.name ="vibrator", //注意这个名字,由于HAL层里面的设备为//"/sys/class/timed_output/vibrator/enable"
	                   //因此这个名字必须为"vibrator"
	.enable= motor_enable,
	.get_time= get_time,
};
 
static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer) //定时器结束时候的回调函数
{
	schedule_work(&vibdata.work); //定时器完成了 执行work队列回调函数来关闭电机
	return HRTIMER_NORESTART;
}
static void vibrator_work(struct work_struct *work)
{
	vibrator_off();
}
 
 
static int motor_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	enum of_gpio_flags flags;
	int ret =0;
	
	hrtimer_init(&vibdata.timer,CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function= vibrator_timer_func;
	INIT_WORK(&vibdata.work,vibrator_work);

	pr_info("%s enter: %d\n", __func__, __LINE__);
	if (!node)
        return -ENODEV;
    gpio = of_get_named_gpio_flags(node, "en-gpio", 0, &flags);
    en_value = (flags == GPIO_ACTIVE_HIGH) ? 1 : 0;
    if (!gpio_is_valid(gpio)) {
        dev_err(&pdev->dev, "invalid en gpio%d\n", gpio);
    }
    printk("%s enter: LINE-%d,GPIO-%d\n", __func__, __LINE__,gpio);
    ret = devm_gpio_request(&pdev->dev, gpio, "motor-en-gpio");

    if (ret) {
        dev_err(&pdev->dev,
                "failed to request GPIO%d for motor-en-gpio\n",
                gpio);
        return -EINVAL;
    }
    gpio_direction_output(gpio, !en_value);

	wake_lock_init(&vibdata.wklock,WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&vibdata.lock);
	ret=timed_output_dev_register(&motot_driver);
	if (ret< 0)
		goto err_to_dev_reg;
	return 0;

err_to_dev_reg:
	mutex_destroy(&vibdata.lock);
	wake_lock_destroy(&vibdata.wklock);

	printk("vibrator   err!:%d\n",ret);
	return ret;

}
static int motor_remove(struct platform_device *pdev)
{
	mutex_destroy(&vibdata.lock);
	wake_lock_destroy(&vibdata.wklock);
	printk("vibrator  exit!\n");
	timed_output_dev_unregister(&motot_driver);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int motor_suspend(struct device *dev)
{
    return 0;
}

static int motor_resume(struct device *dev)
{
    //pr_info("%s: %d\n", __func__, __LINE__);
    return 0;
}
#endif

static const struct dev_pm_ops motor_pm_ops = {
#ifdef CONFIG_PM_SLEEP
    .suspend = motor_suspend,
    .resume = motor_resume,
    .poweroff = motor_suspend,
    .restore = motor_resume,
#endif
};

static struct of_device_id motor_of_match[] = {
    { .compatible = "simple-motor" },
    { }
};

static struct platform_driver motor_driver = {
    .driver		= {
        .name	= "simple-motor",
        .owner	= THIS_MODULE,
        .pm		= &motor_pm_ops,
        .of_match_table	= of_match_ptr(motor_of_match),
    },
    .probe		= motor_probe,
    .remove		= motor_remove,
};

module_platform_driver(motor_driver);
 
MODULE_AUTHOR("jimmy <lijiaming@knowin.com>");
MODULE_DESCRIPTION("Motor Vibrator driver");
MODULE_LICENSE("GPL");