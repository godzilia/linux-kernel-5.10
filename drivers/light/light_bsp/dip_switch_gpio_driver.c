#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_platform.h>

/* 驱动私有数据结构 */
struct dip_switch_gpio_data {
    int gpio_switch;            /* 拨码开关连接的GPIO引脚 */
    int irq;                    /* 中断号 */
    bool active_low;            /* 低电平有效标志 */
    bool use_irq;               /* 是否使用中断检测 */
    struct work_struct work;    /* 工作队列处理电源状态切换 */
    struct delayed_work poll_work; /* 轮询检测工作队列 */
    struct platform_device *pdev;  /* 平台设备指针 */
    struct regulator *regulator;   /* 电源调节器句柄 */
    bool is_running;            /* 驱动运行状态标志 */
    spinlock_t lock;            /* 保护状态变量的自旋锁 */
    atomic_t work_running;      /* 工作队列运行计数器 */
    bool power_state;           /* 电源状态：true=打开，false=关闭 */
    bool switch_state;          /* 开关状态：true=打开，false=关闭 */
};

/* 工作队列处理函数 - 切换电源状态 */
static void dip_switch_power_control_work(struct work_struct *work);

/* 中断处理函数 */
static irqreturn_t dip_switch_irq_handler(int irq, void *dev_id);

/* 轮询检测函数 */
static void dip_switch_poll(struct work_struct *work);

/* 工作队列处理函数 - 切换电源状态 */
static void dip_switch_power_control_work(struct work_struct *work)
{
    struct dip_switch_gpio_data *data = container_of(work, struct dip_switch_gpio_data, work);
    bool current_state;
    
    /* 获取当前开关状态 */
    current_state = (gpio_get_value(data->gpio_switch) == (data->active_low ? 0 : 1));
    
    
    /* 如果开关状态发生变化 */
    if (current_state != data->switch_state) {
        data->switch_state = current_state;
        
        /* 根据开关状态控制电源 */
        if (current_state) {
            /* 开关打开，打开电源 */
            if (data->regulator) {
                int ret = regulator_enable(data->regulator);
                if (ret) {
                    dev_err(&data->pdev->dev, "打开电源失败: %d\n", ret);
                } else {
                    data->power_state = true;
                    dev_info(&data->pdev->dev, "拨码开关打开，已打开电源\n");
                }
            } else {
                dev_warn(&data->pdev->dev, "无可用电源调节器，无法打开电源\n");
            }
        } else {
            /* 开关关闭，关闭电源 */
            if (data->regulator) {
                int ret = regulator_disable(data->regulator);
                if (ret) {
                    dev_err(&data->pdev->dev, "关闭电源失败: %d\n", ret);
                } else {
                    data->power_state = false;
                    dev_info(&data->pdev->dev, "拨码开关关闭，已关闭电源\n");
                }
            } else {
                dev_warn(&data->pdev->dev, "无可用电源调节器，无法关闭电源\n");
            }
        }
    }
    
}

/* 中断处理函数 */
static irqreturn_t dip_switch_irq_handler(int irq, void *dev_id)
{
    struct dip_switch_gpio_data *data = dev_id;
    unsigned long flags;
    
    spin_lock_irqsave(&data->lock, flags);
    
    /* 检查驱动是否仍在运行 */
    if (!data->is_running) {
        spin_unlock_irqrestore(&data->lock, flags);
        return IRQ_HANDLED;
    }
    
    spin_unlock_irqrestore(&data->lock, flags);
    
    /* 调度工作队列处理电源状态切换 */
    schedule_work(&data->work);
    
    return IRQ_HANDLED;
}

/* 轮询检测函数 */
static void dip_switch_poll(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
    struct dip_switch_gpio_data *data = container_of(dwork, struct dip_switch_gpio_data, poll_work);
    unsigned long flags;
    int running;
    /* 增加工作队列运行计数 */
    running = atomic_inc_return(&data->work_running) - 1;
    
    spin_lock_irqsave(&data->lock, flags);
    
    /* 检查驱动是否仍在运行 */
    if (!data->is_running) {
        spin_unlock_irqrestore(&data->lock, flags);
        atomic_dec(&data->work_running);
        return;
    }
    
    spin_unlock_irqrestore(&data->lock, flags);
    /* 检查拨码开关状态 */
    if (gpio_get_value(data->gpio_switch) != (data->switch_state ? 
            (data->active_low ? 0 : 1) : (data->active_low ? 1 : 0))) {
        /* 状态发生变化，调度工作队列处理电源状态切换 */
        schedule_work(&data->work);
    }
    
    /* 重新调度轮询 */
    spin_lock_irqsave(&data->lock, flags);
    if (data->is_running) {
        if (!schedule_delayed_work(&data->poll_work, msecs_to_jiffies(100))) {
            dev_warn(&data->pdev->dev, "轮询工作队列调度失败\n");
        }
    }
    spin_unlock_irqrestore(&data->lock, flags);
    
    /* 减少工作队列运行计数 */
    atomic_dec(&data->work_running);
}

/* 平台驱动probe函数 */
static int dip_switch_gpio_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct dip_switch_gpio_data *data;
    struct device_node *np = dev->of_node;
    struct device_node *power_np = NULL;
    const char *power_name = NULL;
    int ret;

    /* 分配并初始化驱动数据结构 */
    data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
    if (!data) {
        dev_err(dev, "内存分配失败\n");
        return -ENOMEM;
    }
    
    platform_set_drvdata(pdev, data);
    data->pdev = pdev;
    data->is_running = true;  /* 初始化运行状态 */
    spin_lock_init(&data->lock);  /* 初始化自旋锁 */
    atomic_set(&data->work_running, 0);  /* 初始化工作队列计数器 */
    
    /* 从设备树获取拨码开关GPIO配置 */
    data->gpio_switch = of_get_named_gpio(np, "switch-gpios", 0);
    if (!gpio_is_valid(data->gpio_switch)) {
        dev_err(dev, "无效的拨码开关GPIO\n");
        return -ENODEV;
    }
    
    /* 获取设备树中的其他属性 */
    data->active_low = of_property_read_bool(np, "active-low");
    data->use_irq = of_property_read_bool(np, "use-irq");
    
    /* 从设备树获取电源供应器引用 */
    if (of_find_property(np, "power-supply", NULL)) {
        /* 解析power-supply引用 */
        power_np = of_parse_phandle(np, "power-supply", 0);
        if (!power_np) {
            dev_err(dev, "无法解析power-supply引用\n");
            return -EINVAL;
        }
        
        /* 获取电源调节器名称 */
        ret = of_property_read_string(power_np, "regulator-name", &power_name);
        if (ret) {
            /* 如果没有regulator-name属性，尝试使用节点名称 */
            power_name = power_np->name;
            dev_info(dev, "使用节点名称作为调节器名称: %s\n", power_name);
        } else {
            dev_info(dev, "使用属性regulator-name: %s\n", power_name);
        }
        
        /* 获取电源调节器 */
        data->regulator = devm_regulator_get(dev, power_name);
        if (IS_ERR(data->regulator)) {
            ret = PTR_ERR(data->regulator);
            dev_err(dev, "无法获取电源调节器 %s: %d\n", power_name, ret);
            of_node_put(power_np);
            return ret;
        }
        
        dev_info(dev, "已连接电源调节器: %s\n", power_name);
        of_node_put(power_np);  /* 释放节点引用 */
    } else {
        dev_warn(dev, "设备树中未指定power-supply属性\n");
        data->regulator = NULL;
    }
    
    /* 申请GPIO */
    ret = devm_gpio_request_one(dev, data->gpio_switch, 
                               data->use_irq ? GPIOF_IN : GPIOF_IN | GPIOF_OPEN_DRAIN,
                               "dip_switch_switch");
    if (ret) {
        dev_err(dev, "无法申请拨码开关GPIO\n");
        return ret;
    }
    
    /* 初始化工作队列 */
    INIT_WORK(&data->work, dip_switch_power_control_work);
    
    /* 读取初始开关状态 */
    data->switch_state = (gpio_get_value(data->gpio_switch) == (data->active_low ? 0 : 1));
    
    /* 初始化电源状态 */
    data->power_state = data->switch_state;
    
    /* 根据初始开关状态设置电源 */
    if (data->regulator) {
        if (data->switch_state) {
            /* 开关打开，打开电源 */
            int ret = regulator_enable(data->regulator);
            if (ret) {
                dev_err(dev, "初始化时打开电源失败: %d\n", ret);
            } else {
                dev_info(dev, "初始化: 拨码开关打开，已打开电源\n");
            }
        } else {
            /* 开关关闭，关闭电源 */
            int ret = regulator_disable(data->regulator);
            if (ret) {
                dev_err(dev, "初始化时关闭电源失败: %d\n", ret);
            } else {
                dev_info(dev, "初始化: 拨码开关关闭，已关闭电源\n");
            }
        }
    }
    
    if (data->use_irq) {
        /* 申请中断 */
        data->irq = gpio_to_irq(data->gpio_switch);
        if (data->irq < 0) {
            dev_err(dev, "无法获取拨码开关GPIO的中断号\n");
            return data->irq;
        }
        
        ret = devm_request_irq(dev, data->irq, dip_switch_irq_handler,
                              IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, /* 双边沿触发 */
                              "dip_switch_irq", data);
        if (ret) {
            dev_err(dev, "无法申请中断\n");
            return ret;
        }
        
        dev_info(dev, "使用中断检测模式，GPIO: %d, 中断号: %d\n", 
                data->gpio_switch, data->irq);
    } else {
        /* 使用轮询模式 */
        INIT_DELAYED_WORK(&data->poll_work, dip_switch_poll);
        
        /* 调度初始轮询 */
        if (!schedule_delayed_work(&data->poll_work, msecs_to_jiffies(100))) {
            dev_warn(dev, "初始轮询工作队列调度失败\n");
        }
        
        dev_info(dev, "使用轮询检测模式，GPIO: %d\n", data->gpio_switch);
    }
    
    dev_info(dev, "拨码开关GPIO驱动初始化成功\n");
    return 0;
}

/* 平台驱动remove函数 */
static int dip_switch_gpio_remove(struct platform_device *pdev)
{
    struct dip_switch_gpio_data *data = platform_get_drvdata(pdev);
    unsigned long flags;
    int timeout = 1000;  /* 超时时间(毫秒) */
    
    dev_info(&pdev->dev, "正在移除拨码开关GPIO驱动...\n");
    
    /* 标记驱动为停止状态 */
    spin_lock_irqsave(&data->lock, flags);
    data->is_running = false;
    spin_unlock_irqrestore(&data->lock, flags);
    
    /* 等待所有工作队列完成 */
    while (atomic_read(&data->work_running) > 0 && timeout > 0) {
        msleep(10);
        timeout -= 10;
    }
    
    if (timeout <= 0) {
        dev_warn(&pdev->dev, "等待工作队列超时，强制取消\n");
    }
    
    /* 取消调度工作队列 */
    if (!data->use_irq) {
        cancel_delayed_work_sync(&data->poll_work);
    }
    
    cancel_work_sync(&data->work);
    
    /* 关闭电源 */
    if (data->regulator && data->power_state) {
        regulator_disable(data->regulator);
        dev_info(&pdev->dev, "驱动卸载，已关闭电源\n");
    }
    
    dev_info(&pdev->dev, "拨码开关GPIO驱动已移除\n");
    return 0;
}

/* 设备树匹配表 */
static const struct of_device_id dip_switch_gpio_of_match[] = {
    { .compatible = "zhihe,dip-switch", },
    {},
};
MODULE_DEVICE_TABLE(of, dip_switch_gpio_of_match);

/* 平台驱动结构 */
static struct platform_driver dip_switch_gpio_driver = {
    .probe      = dip_switch_gpio_probe,
    .remove     = dip_switch_gpio_remove,
    .driver     = {
        .name   = "dip-switch-gpio",
        .of_match_table = dip_switch_gpio_of_match,
        .owner  = THIS_MODULE,
    },
};

/* 模块初始化和退出函数 */
static int __init dip_switch_gpio_init(void)
{
    return platform_driver_register(&dip_switch_gpio_driver);
}

static void __exit dip_switch_gpio_exit(void)
{
    platform_driver_unregister(&dip_switch_gpio_driver);
}

module_init(dip_switch_gpio_init);
module_exit(dip_switch_gpio_exit);

MODULE_DESCRIPTION("使用设备树电源引用的拨码开关GPIO驱动");
MODULE_AUTHOR("huangxl");
MODULE_LICENSE("GPL");
