#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/syscore_ops.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/workqueue.h>

#define EC_REG_START   					 (EC_MAIN_VERSION)   				 /* 起始寄存器地址 */
#define EC_REG_END     					 (EC_BAT_FULL_CHARGE_CAPACITY_H)     /* 结束寄存器地址 */
#define EC_MAX_RETRIES 					 (3)      							 /* 操作重试次数 */

#define EC_MAIN_VERSION  				 (0x00)								 
#define EC_SUB_VERSION   				 (0x01)
#define EC_SOC_ACPI_STATUS               (0x0A)
#define EC_POWER_STATUS  				 (0x20)  // 电源状态寄存器
#define EC_BAT_REMAIN_PERCENT			 (0x21)
#define EC_BAT_REMAIN_CAPACITY_L		 (0x22)
#define EC_BAT_REMAIN_CAPACITY_H		 (0x23)
#define EC_BAT_DESIGN_CAPACITY_L		 (0x24)
#define EC_BAT_DESIGN_CAPACITY_H		 (0x25)
#define EC_BAT_PRESENT_VOLTAGE_L		 (0x26)
#define EC_BAT_PRESENT_VOLTAGE_H		 (0x27)
#define EC_BAT_DESIGN_VOLTAGE_L	 		 (0x28)
#define EC_BAT_DESIGN_VOLTAGE_H	 		 (0x29)
#define EC_BAT_PRESENT_CURRENT_L	 	 (0x2A)
#define EC_BAT_PRESENT_CURRENT_H		 (0x2B)
#define EC_BAT_FULL_CHARGE_CAPACITY_L	 (0x2C)
#define EC_BAT_FULL_CHARGE_CAPACITY_H	 (0x2D)

/* 新增寄存器定义 */
#define EC_QEVENT_DATA                 (0x10)    /* Q事件数据寄存器 */
#define EC_LID_STATUS                  (0x06)    /* 盖子状态寄存器 */

/* 新增事件类型定义 */
#define QEVENT_POWER                   (0xB4)    /* 电源事件 */
#define QEVENT_LID                     (0xD0)    /* 盖子事件 */

enum e_soc_acpi_status {
	ACPI_WORK 	  = 0x0,      //work  S0
	ACPI_S1 	  = 0x1,
	ACPI_S2 	  = 0x2,
	ACPI_STR 	  = 0x3,      //str
	ACPI_STD 	  = 0x4,      //std
	ACPI_POWEROFF = 0x5,      //poweroff
	ACPI_REBOOT   = 0x6,      //reboot
};

/* 电源适配器状态枚举 */
enum power_adapter_status {
	POWER_ADAPTER_NOT_PRESENT = 0,
	POWER_ADAPTER_PRESENT = 1,
};

/* 电池存在状态枚举 */
enum battery_present_status {
	BATTERY_NOT_PRESENT = 0,
	BATTERY_PRESENT = 1,
};

/* 电池充电状态枚举 */
enum battery_charge_status {
	BATTERY_DISCHARGING = 0,
	BATTERY_CHARGING = 1,
};

/* 电池电量状态枚举 */
enum battery_power_status {
	BATTERY_POWER_NORMAL = 0,
	BATTERY_POWER_DRAINED = 1,
};

/* 电池满充状态枚举 */
enum battery_full_status {
	BATTERY_NOT_FULL = 0,
	BATTERY_FULL = 1,
};

/* 电池状态枚举 */
enum battery_status {
	BATTERY_STATUS_UNKNOWN = 0,
	BATTERY_STATUS_CHARGING = 1,
	BATTERY_STATUS_DISCHARGING = 2,
	BATTERY_STATUS_FULL = 3,
	BATTERY_STATUS_NOT_CHARGING = 4,
	BATTERY_STATUS_POWER_DRAINED = 5,
};

/* 盖子状态枚举 */
enum lid_status {
    LID_OPEN = 0,
    LID_CLOSED = 1,
};

/* 工作队列参数结构体 */
struct wakeup_work_data {
    struct work_struct work;
    struct ec_device *ec;
    u8 qevent_data;
};

struct ec_device {
    struct i2c_client *client;
    struct regmap *regmap;
    struct mutex lock;
    struct power_supply *battery_psy;  /* 电池电源供应器 */
    struct delayed_work battery_work;  /* 电池数据刷新工作 */
    bool battery_enabled;              /* 电池功能启用标志 */
    
    /* 电池状态缓存 */
    unsigned int battery_percent;
    unsigned int battery_voltage;
    int battery_current;
    unsigned int battery_capacity;
    unsigned int battery_design_capacity;
    enum battery_status battery_status;
    
    /* 电源适配器状态 */
    enum power_adapter_status adapter_status;
    enum battery_present_status battery_present;
    enum battery_charge_status charge_status;
    enum battery_power_status power_status;
    enum battery_full_status full_status;
    
    /* 输入事件处理 */
    struct input_handler wakeup_handler;    
    struct input_dev *input_dev;             /* 用于上报SW_LID事件的输入设备 */
    
    /* 工作队列相关 */
    struct workqueue_struct *wakeup_wq;
    struct wakeup_work_data wakeup_work;
};

struct ec_device *ec;

/* 函数前向声明 */
static void handle_lid_event(struct ec_device *ec);
static void wakeup_event(struct input_handle *handle, unsigned int type,
                       unsigned int code, int value);
static int wakeup_connect(struct input_handler *handler, struct input_dev *dev,
                         const struct input_device_id *id);
static void wakeup_disconnect(struct input_handle *handle);
static void wakeup_work_handler(struct work_struct *work);

/* EC寄存器读写状态回调 */
static bool ec_reg_volatile(struct device *dev, unsigned int reg)
{
    /* 某些寄存器可能是易变的，例如状态寄存器 */
    if (reg == EC_SOC_ACPI_STATUS || reg == EC_POWER_STATUS || reg == EC_QEVENT_DATA)
        return true;
    return false;
}

/* EC寄存器读写权限配置 */
static bool ec_reg_readable(struct device *dev, unsigned int reg)
{
    /* 检查寄存器是否可读 */
    return (reg >= EC_REG_START && reg <= EC_REG_END);
}

static bool ec_reg_writeable(struct device *dev, unsigned int reg)
{
    /* 检查寄存器是否可写 */
    if (reg == EC_SOC_ACPI_STATUS)
        return true;  /* 例如，命令寄存器和控制寄存器可写 */
    return false;
}

/* 读取单个寄存器 */
static int ec_read_reg(struct ec_device *ec, u8 reg, u8* val)
{
    int ret;
    u32 regmap_val = 0;
    mutex_lock(&ec->lock);
    ret = regmap_read(ec->regmap, reg, &regmap_val);
    mutex_unlock(&ec->lock);
    *val = (u8)regmap_val;
    return ret;
}

/* 写入单个寄存器 */
static int ec_write_reg(struct ec_device *ec, u8 reg, u8 val)
{
    int ret;
    
    mutex_lock(&ec->lock);
    ret = regmap_write(ec->regmap, reg, val);
    mutex_unlock(&ec->lock);
    
    return ret;
}

/* 读取电池百分比 */
static int read_battery_percent(struct ec_device *ec)
{
    u8 percent;
    int ret;
    
    ret = ec_read_reg(ec, EC_BAT_REMAIN_PERCENT, &percent);
    if (ret)
        return ret;
        
    ec->battery_percent = percent;
    return 0;
}

/* 读取电池电压 */
static int read_battery_voltage(struct ec_device *ec)
{
    u8 voltage_l, voltage_h;
    int ret;
    
    ret = ec_read_reg(ec, EC_BAT_PRESENT_VOLTAGE_L, &voltage_l);
    if (ret)
        return ret;
        
    ret = ec_read_reg(ec, EC_BAT_PRESENT_VOLTAGE_H, &voltage_h);
    if (ret)
        return ret;
        
    /* 转换为毫伏 (电压分辨率为1mV) */
    ec->battery_voltage = (voltage_h << 8) | voltage_l;
    return 0;
}

/* 读取电池电流 */
static int read_battery_current(struct ec_device *ec)
{
    u8 current_l, current_h;
    int ret;
    int16_t batt_current;  /* 重命名变量，避免与内核宏冲突 */
    
    ret = ec_read_reg(ec, EC_BAT_PRESENT_CURRENT_L, &current_l);
    if (ret)
        return ret;
        
    ret = ec_read_reg(ec, EC_BAT_PRESENT_CURRENT_H, &current_h);
    if (ret)
        return ret;
        
    /* 转换为毫安 (电流分辨率为1mA，有符号数) */
    batt_current = (current_h << 8) | current_l;
    ec->battery_current = batt_current;
    
    return 0;
}

/* 读取电池容量 */
static int read_battery_capacity(struct ec_device *ec)
{
    u8 capacity_l, capacity_h;
    u8 design_cap_l, design_cap_h;
    int ret;
    
    ret = ec_read_reg(ec, EC_BAT_REMAIN_CAPACITY_L, &capacity_l);
    if (ret)
        return ret;
        
    ret = ec_read_reg(ec, EC_BAT_REMAIN_CAPACITY_H, &capacity_h);
    if (ret)
        return ret;
        
    ret = ec_read_reg(ec, EC_BAT_DESIGN_CAPACITY_L, &design_cap_l);
    if (ret)
        return ret;
        
    ret = ec_read_reg(ec, EC_BAT_DESIGN_CAPACITY_H, &design_cap_h);
    if (ret)
        return ret;
        
    /* 转换为毫安时 (电量分辨率为mAh) */
    ec->battery_capacity = (capacity_h << 8) | capacity_l;
    ec->battery_design_capacity = (design_cap_h << 8) | design_cap_l;
    
    return 0;
}

/* 读取电源状态寄存器 */
static int read_power_status(struct ec_device *ec)
{
    u8 power_status;
    int ret;
    
    ret = ec_read_reg(ec, EC_POWER_STATUS, &power_status);
	//dev_info(&ec->client->dev, "EC_POWER_STATUS 0x%02X\n", power_status);
    if (ret)
        return ret;
        
    /* 解析电源状态寄存器 */
    ec->adapter_status = (power_status & (1 << 0)) ? POWER_ADAPTER_PRESENT : POWER_ADAPTER_NOT_PRESENT;
    ec->battery_present = (power_status & (1 << 1)) ? BATTERY_PRESENT : BATTERY_NOT_PRESENT;
    ec->charge_status = (power_status & (1 << 2)) ? BATTERY_CHARGING : BATTERY_DISCHARGING;
    ec->power_status = (power_status & (1 << 3)) ? BATTERY_POWER_DRAINED : BATTERY_POWER_NORMAL;
    ec->full_status = (power_status & (1 << 4)) ? BATTERY_FULL : BATTERY_NOT_FULL;
    
    return 0;
}

/* 刷新电池数据 */
static int refresh_battery_data(struct ec_device *ec)
{
    int ret;
    
    /* 读取所有电池相关数据 */
    ret = read_battery_percent(ec);
    if (ret)
        return ret;
        
    ret = read_battery_voltage(ec);
    if (ret)
        return ret;
        
    ret = read_battery_current(ec);
    if (ret)
        return ret;
        
    ret = read_battery_capacity(ec);
    if (ret)
        return ret;
        
    ret = read_power_status(ec);
    if (ret)
        return ret;
        
    return 0;
}

/* 更新电池状态 */
static void update_battery_status(struct ec_device *ec)
{
    /* 根据电源状态寄存器更新电池状态 */
    if (!ec->battery_present) {
        ec->battery_status = BATTERY_STATUS_UNKNOWN;
        return;
    }
    
    if (ec->power_status == BATTERY_POWER_DRAINED) {
        ec->battery_status = BATTERY_STATUS_POWER_DRAINED;
        return;
    }
    
    if (ec->full_status == BATTERY_FULL) {
        ec->battery_status = BATTERY_STATUS_FULL;
        return;
    }
    
    if (ec->charge_status == BATTERY_CHARGING) {
        ec->battery_status = BATTERY_STATUS_CHARGING;
    } else {
        ec->battery_status = BATTERY_STATUS_DISCHARGING;
    }
}

/* 电池数据刷新工作队列处理函数 */
static void battery_work_handler(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct ec_device *ec = container_of(dwork, struct ec_device, battery_work);
    int ret;
    
    if (!ec->battery_enabled)
        return;
        
    ret = refresh_battery_data(ec);
    if (ret) {
        dev_err(&ec->client->dev, "Failed to refresh battery data: %d\n", ret);
    } else {
        /* 更新电池状态 */
        update_battery_status(ec);
        
        /* 通知电源供应器子系统数据已更新 */
        power_supply_changed(ec->battery_psy);
    }
    
    /* 安排下一次刷新 */
    schedule_delayed_work(&ec->battery_work, msecs_to_jiffies(1000));
}

/* 唤醒事件工作队列处理函数 - 在非原子上下文执行 */
static void wakeup_work_handler(struct work_struct *work)
{
    struct wakeup_work_data *data = container_of(work, struct wakeup_work_data, work);
    struct ec_device *ec = data->ec;
    u8 qevent_data = data->qevent_data;
    int ret;
    
    // 打印事件处理函数入口
    //printk(KERN_INFO "wakeup_work_handler: QEvent data=0x%02X\n", qevent_data);
    
    // 读取Q事件数据寄存器（工作队列中执行）
    ret = ec_read_reg(ec, EC_QEVENT_DATA, &qevent_data);
    if (ret) {
        dev_err(&ec->client->dev, "Failed to read EC_QEVENT_DATA in work queue: %d\n", ret);
        return;
    }
    
    dev_info(&ec->client->dev, "QEvent data processed: 0x%02X\n", qevent_data);
    
    // 处理不同类型的Q事件
    switch (qevent_data) {
        case QEVENT_POWER:
            //dev_info(&ec->client->dev, "Power event detected, refreshing battery status\n");
            // 立即触发电池状态刷新
               /* 取消并等待当前延迟工作完成 */
		    cancel_delayed_work_sync(&ec->battery_work);
		    
		    /* 立即提交工作执行 */
		    schedule_work(&ec->battery_work.work);
            break;
            
        case QEVENT_LID:
            dev_info(&ec->client->dev, "Lid event detected, checking lid status\n");
            handle_lid_event(ec);
            break;
            
        default:
            dev_info(&ec->client->dev, "Unhandled QEvent: 0x%02X\n", qevent_data);
            break;
    }
}

/* 唤醒事件处理函数 - 仅将工作添加到队列 */
static void wakeup_event(struct input_handle *handle, unsigned int type,
                       unsigned int code, int value)
{
    struct ec_device *ec = handle->private;  /* 从handle直接获取ec */
    
    // 打印事件处理函数入口
    printk(KERN_INFO "Enter wakeup_event, type=%u, code=%u, value=%d\n", type, code, value);
    
    // 只处理按键按下事件
    if (type == EV_KEY) {
        printk(KERN_INFO "EV_KEY event detected, code=%u, value=%d\n", code, value);
        
        if (code == KEY_WAKEUP && value == 1) {
            dev_info(&ec->client->dev, "KEY_WAKEUP event detected, queuing for processing\n");
            
            /* 将事件处理放入工作队列，确保在非原子上下文执行 */
            ec->wakeup_work.ec = ec;
            ec->wakeup_work.qevent_data = 0; // 工作队列中读取实际值
            queue_work(ec->wakeup_wq, &ec->wakeup_work.work);
        }
    }
}

/* 处理盖子事件 */
static void handle_lid_event(struct ec_device *ec)
{
    u8 lid_status;
    int ret;
    
    // 读取盖子状态寄存器
    ret = ec_read_reg(ec, EC_LID_STATUS, &lid_status);
    if (ret) {
        dev_err(&ec->client->dev, "Failed to read EC_LID_STATUS: %d\n", ret);
        return;
    }
    
    dev_info(&ec->client->dev, "Lid status: %s\n", 
            lid_status == LID_CLOSED ? "closed" : "open");
    
    // 上报SW_LID事件
    if (ec->input_dev) {
        input_report_switch(ec->input_dev, SW_LID, lid_status == LID_CLOSED);
        input_sync(ec->input_dev);
        dev_info(&ec->client->dev, "SW_LID event reported: %s\n", 
                lid_status == LID_CLOSED ? "closed" : "open");
    }
}

/* 唤醒事件处理器连接回调 - 添加详细打印 */
static int wakeup_connect(struct input_handler *handler, struct input_dev *dev,
                         const struct input_device_id *id)
{
    // 添加编译时类型检查
    BUILD_BUG_ON_MSG(!__same_type(*handler, ((struct ec_device *)0)->wakeup_handler),
                     "wakeup_handler type mismatch");
                     
    struct input_handle *handle;
    struct ec_device *ec = container_of(handler, struct ec_device, wakeup_handler);
    
    handle = devm_kzalloc(&ec->client->dev, sizeof(*handle), GFP_KERNEL);
    if (!handle) {
        dev_err(&ec->client->dev, "Failed to allocate input handle\n");
        return -ENOMEM;
    }
    
    handle->dev = dev;
    handle->handler = handler;
    handle->name = "ec-wakeup-handle";
    handle->private = ec;  /* 存储ec指针到handle私有数据 */
    
    // 设置设备私有数据
    input_set_drvdata(dev, ec);
    
    // 打印连接的设备信息
    dev_info(&ec->client->dev, "Connected to input device: %s\n", dev->name);
    printk(KERN_INFO "Device %s has EV_KEY: %d\n", dev->name, test_bit(EV_KEY, dev->evbit));
    printk(KERN_INFO "Device %s has KEY_WAKEUP: %d\n", dev->name, test_bit(KEY_WAKEUP, dev->keybit));
    
    int ret = input_register_handle(handle);
    if (ret) {
        dev_err(&ec->client->dev, "Failed to register input handle: %d\n", ret);
        devm_kfree(&ec->client->dev, handle);
        return ret;
    }
    ret = input_open_device(handle);
    if (ret) {
        dev_err(&ec->client->dev, "Failed to open input device: %d\n", ret);
        input_unregister_handle(handle);
        kfree(handle);
        return ret;
    }
    dev_info(&ec->client->dev, "Successfully connected to input device: %s\n", dev->name);
    return 0;
}

/* 唤醒事件处理器断开回调 */
static void wakeup_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
    input_unregister_handle(handle);
    dev_info(&handle->dev->dev, "Disconnected from input device\n");
}

/* 唤醒事件处理器匹配表 */
static const struct input_device_id wakeup_ids[] = {
    {
        .flags = INPUT_DEVICE_ID_MATCH_EVBIT | INPUT_DEVICE_ID_MATCH_KEYBIT,
        .evbit = { BIT_MASK(EV_KEY) },
        .keybit = { [BIT_WORD(KEY_WAKEUP)] = BIT_MASK(KEY_WAKEUP) },
    },
    { }, // 终止符
};

/* 初始化唤醒事件监听 - 添加详细打印 */
static int init_wakeup_event_listener(struct ec_device *ec)
{
    // 直接初始化结构体成员，无需分配内存
    ec->wakeup_handler.event = wakeup_event;
    ec->wakeup_handler.connect = wakeup_connect;
    ec->wakeup_handler.disconnect = wakeup_disconnect;
    ec->wakeup_handler.name = "ec-wakeup-handler";
    ec->wakeup_handler.id_table = wakeup_ids;
    
    int ret = input_register_handler(&ec->wakeup_handler);
    if (ret) {
        dev_err(&ec->client->dev, "Failed to register wakeup event handler: %d\n", ret);
        return ret;
    }
    
    dev_info(&ec->client->dev, "Wakeup event handler registered successfully\n");
    return 0;
}

/* 初始化用于上报SW_LID事件的输入设备 - 添加打印 */
static int init_lid_input_device(struct ec_device *ec)
{
    struct input_dev *input_dev;
    int ret;
    
    // 分配输入设备
    input_dev = devm_input_allocate_device(&ec->client->dev);
    if (!input_dev) {
        dev_err(&ec->client->dev, "Failed to allocate lid input device\n");
        return -ENOMEM;
    }
    
    // 设置设备属性
    input_dev->name = "ec-lid-switch";
    input_dev->phys = "ec-lid/input0";
    input_dev->id.bustype = BUS_HOST;
    
    // 设置支持的事件类型和事件代码
    __set_bit(EV_SW, input_dev->evbit);
    __set_bit(SW_LID, input_dev->swbit);
    
    // 注册输入设备
    ret = input_register_device(input_dev);
    if (ret) {
        dev_err(&ec->client->dev, "Failed to register lid input device: %d\n", ret);
        return ret;
    }
    
    ec->input_dev = input_dev;
    dev_info(&ec->client->dev, "Lid input device registered successfully: %s\n", input_dev->name);
    return 0;
}

/* 电池属性获取回调函数 */
static int battery_get_property(struct power_supply *psy,
                               enum power_supply_property psp,
                               union power_supply_propval *val)
{
    struct ec_device *ec = power_supply_get_drvdata(psy);
    int ret = 0;
    
    if (!ec->battery_enabled)
        return -ENOMEM;
        
    switch (psp) {
        case POWER_SUPPLY_PROP_CAPACITY:
            val->intval = ec->battery_percent;
            break;
            
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
            val->intval = ec->battery_voltage * 1000; /* 转换为微伏 */
            break;
            
        case POWER_SUPPLY_PROP_CURRENT_NOW:
            val->intval = ec->battery_current * 1000; /* 转换为微安 */
            break;
            
        case POWER_SUPPLY_PROP_STATUS:
            switch (ec->battery_status) {
                case BATTERY_STATUS_CHARGING:
                    val->intval = POWER_SUPPLY_STATUS_CHARGING;
                    break;
                case BATTERY_STATUS_DISCHARGING:
                    val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
                    break;
                case BATTERY_STATUS_FULL:
                    val->intval = POWER_SUPPLY_STATUS_FULL;
                    break;
                case BATTERY_STATUS_NOT_CHARGING:
                    val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
                    break;
                case BATTERY_STATUS_POWER_DRAINED:
                    val->intval = POWER_SUPPLY_STATUS_DISCHARGING; /* 使用DISCHARGING代替EMPTY */
                    break;
                default:
                    val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
                    break;
            }
            break;
            
        case POWER_SUPPLY_PROP_PRESENT:
            val->intval = ec->battery_present;
            break;
            
        case POWER_SUPPLY_PROP_ONLINE:
            val->intval = ec->adapter_status;
            break;
            
        case POWER_SUPPLY_PROP_ENERGY_NOW:
            /* 将容量转换为毫瓦时 (假设平均电压为3.7V) */
            val->intval = ec->battery_capacity * 3700;
            break;
            
        case POWER_SUPPLY_PROP_ENERGY_FULL:
            /* 将设计容量转换为毫瓦时 (假设平均电压为3.7V) */
            val->intval = ec->battery_design_capacity * 3700;
            break;
            
        default:
            ret = -EINVAL;
            break;
    }
    
    return ret;
}

/* 电池支持的属性列表 */
static enum power_supply_property battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_ENERGY_NOW,
    POWER_SUPPLY_PROP_ENERGY_FULL,
};

/* 电池电源供应器描述 */
static const struct power_supply_desc battery_desc = {
    .name = "ec-battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .properties = battery_props,
    .num_properties = ARRAY_SIZE(battery_props),
    .get_property = battery_get_property,
};

/* 设备挂起回调 */
static int ec_suspend(struct device *dev)
{
    struct ec_device *ec = dev_get_drvdata(dev);
    int ret = 0;
    u8 val = 0;

    dev_info(dev, "Suspending EC device...\n");
    
    /* 取消电池数据刷新工作 */
    cancel_delayed_work_sync(&ec->battery_work);
    
    /* 向EC发送挂起命令 */
    ret = ec_write_reg(ec, EC_SOC_ACPI_STATUS, ACPI_STR);
    if (ret) {
        dev_err(dev, "Failed to send suspend command: %d\n", ret);
        return ret;
    }
    
    ec_read_reg(ec, EC_SOC_ACPI_STATUS, &val);
    dev_info(dev, "%s:write ec EC_SOC_ACPI_STATUS: %d\n", __func__, val);

    return 0;
}

/* 设备恢复回调 */
static int ec_resume(struct device *dev)
{
    struct ec_device *ec = dev_get_drvdata(dev);
    int ret = 0;
    u8 val = 0;

    dev_info(dev, "Resuming EC device...\n");
    
    /* 向EC发送恢复命令 */
    ret = ec_write_reg(ec, EC_SOC_ACPI_STATUS, ACPI_WORK);
    if (ret) {
        dev_err(dev, "Failed to send resume command: %d\n", ret);
        return ret;
    }
    
    ec_read_reg(ec, EC_SOC_ACPI_STATUS, &val);
    dev_info(dev, "%s:write ec EC_SOC_ACPI_STATUS: %d\n", __func__, val);
    
    /* 恢复电池数据刷新 */
    if (ec->battery_enabled)
        schedule_delayed_work(&ec->battery_work, msecs_to_jiffies(1000));
    
    return 0;
}

static SIMPLE_DEV_PM_OPS(ec_pm_ops, ec_suspend, ec_resume);

static void TH1520_syscore_shutdown(void)
{
    struct i2c_client *client = ec->client;  /* 通过全局ec获取client */
    struct device *dev = &client->dev;
    struct ec_device *ec = dev_get_drvdata(dev);  /* 从设备数据获取ec */
    
    int ret = 0;
    u8 val = 0;
    
    if (system_state == SYSTEM_POWER_OFF) {
        printk("poweroff %s\n", __func__);
        ret = ec_write_reg(ec, EC_SOC_ACPI_STATUS, ACPI_POWEROFF);
        if (ret) {
            printk(" @@@@@@@@@@@@ fail power off %s %d \n",
                   __func__, __LINE__);
        }
        mdelay(10);
        ec_read_reg(ec, EC_SOC_ACPI_STATUS, &val);
        printk("%s:write ec EC_SOC_ACPI_STATUS: %d\n", __func__, val);
        mdelay(10);
    } else if (system_state == SYSTEM_RESTART) {
        printk("reboot %s\n", __func__);

        ret = ec_write_reg(ec, EC_SOC_ACPI_STATUS, ACPI_REBOOT);
        if (ret) {
            printk("set red-led error\n");
        }
        mdelay(10);
        ec_read_reg(ec, EC_SOC_ACPI_STATUS, &val);
        printk("%s:write ec EC_SOC_ACPI_STATUS: %d\n", __func__, val);
    }
}

static struct syscore_ops TH1520_syscore_ops = {
    .shutdown = TH1520_syscore_shutdown,
};

/* EC设备探测函数 */
static int ec_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
    struct regmap_config config;
    uint8_t ec_main_version = 0;
    uint8_t ec_sub_version = 0;
    int ret;
    
    /* 分配并初始化设备结构 */
    ec = devm_kzalloc(&client->dev, sizeof(*ec), GFP_KERNEL);
    if (!ec)
        return -ENOMEM;

    ec->client = client;
    mutex_init(&ec->lock);
    i2c_set_clientdata(client, ec);

    /* 创建工作队列 */
    ec->wakeup_wq = create_singlethread_workqueue("ec_wakeup_wq");
    if (!ec->wakeup_wq) {
        dev_err(&client->dev, "Failed to create wakeup workqueue\n");
        kfree(ec);
        return -ENOMEM;
    }
    
    /* 初始化工作项 */
    INIT_WORK(&ec->wakeup_work.work, wakeup_work_handler);

    /* 配置regmap */
    config = (struct regmap_config) {
        .reg_bits = 8,              /* 8位寄存器地址 */
        .val_bits = 8,              /* 8位寄存器值 */
        .max_register = EC_REG_END, /* 最大寄存器地址 */
        .writeable_reg = ec_reg_writeable,
        .readable_reg = ec_reg_readable,
        .volatile_reg = ec_reg_volatile,
        .cache_type = REGCACHE_RBTREE, /* 使用红黑树缓存 */
        .reg_defaults = NULL,
        .num_reg_defaults = 0,
        .read_flag_mask = 0x00,
        .write_flag_mask = 0x00,
        .use_single_read = true,
        .use_single_write = true,
    };
    
    register_syscore_ops(&TH1520_syscore_ops);
    
    /* 初始化regmap */
    ec->regmap = devm_regmap_init_i2c(client, &config);
    if (IS_ERR(ec->regmap)) {
        dev_err(&client->dev, "Failed to initialize regmap\n");
        destroy_workqueue(ec->wakeup_wq);
        kfree(ec);
        return PTR_ERR(ec->regmap);
    }
    
    ec_read_reg(ec, 0x00, &ec_main_version);
    ec_read_reg(ec, 0x01, &ec_sub_version);
    printk(KERN_INFO "EC main read regmap version=0x%x, EC sub_version=0x%x\n", 
               ec_main_version, ec_sub_version);
               
    dev_info(&client->dev, "EC device registered at 0x%02X\n", 
             client->addr);

    /* 初始化唤醒事件监听 */
    ret = init_wakeup_event_listener(ec);
    if (ret) {
        dev_err(&client->dev, "Failed to initialize wakeup event listener: %d\n", ret);
        // 可以选择继续执行，不影响主要功能
    }
    
    /* 初始化盖子状态输入设备 */
    ret = init_lid_input_device(ec);
    if (ret) {
        dev_err(&client->dev, "Failed to initialize lid input device: %d\n", ret);
        // 可以选择继续执行，不影响主要功能
    }
    
    /* 初始化电池功能 */
    ec->battery_enabled = true;
    
    /* 初始化电源供应器配置 */
    struct power_supply_config battery_config = {
        .drv_data = ec,  /* 设置私有数据 */
    };
    
    /* 注册电池电源供应器 */
    ec->battery_psy = power_supply_register(&client->dev, &battery_desc, &battery_config);
    if (IS_ERR(ec->battery_psy)) {
        ret = PTR_ERR(ec->battery_psy);
        dev_err(&client->dev, "Failed to register battery power supply: %d\n", ret);
        ec->battery_enabled = false;
    } else {
        /* 初始化电池数据 */
        ret = refresh_battery_data(ec);
        if (ret) {
            dev_err(&client->dev, "Failed to initialize battery data: %d\n", ret);
        } else {
            /* 更新电池状态 */
            update_battery_status(ec);
            
            /* 初始化并调度电池数据刷新工作 */
            INIT_DELAYED_WORK(&ec->battery_work, battery_work_handler);
            schedule_delayed_work(&ec->battery_work, msecs_to_jiffies(1000));
            
            dev_info(&client->dev, "Battery power supply registered successfully\n");
        }
    }

    return 0;
}

/* EC设备移除函数 */
static int ec_remove(struct i2c_client *client)
{
    struct ec_device *ec = i2c_get_clientdata(client);
    
    /* 取消电池数据刷新工作 */
    cancel_delayed_work_sync(&ec->battery_work);
    
    /* 释放唤醒事件处理器 */
    input_unregister_handler(&ec->wakeup_handler);  /* 直接注销，无需判断 */
    
    /* 释放盖子状态输入设备 */
    if (ec->input_dev) {
        input_unregister_device(ec->input_dev);
        ec->input_dev = NULL;
    }
    
    /* 注销电池电源供应器 */
    if (ec->battery_psy)
        power_supply_unregister(ec->battery_psy);
    
    /* 销毁工作队列 */
    if (ec->wakeup_wq) {
        flush_workqueue(ec->wakeup_wq);
        destroy_workqueue(ec->wakeup_wq);
    }
    
    dev_info(&client->dev, "EC device removed\n");
    return 0;
}

static const struct i2c_device_id ec_id_table[] = {
    { "ec_csce2101", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, ec_id_table);

static struct i2c_driver ec_driver = {
    .driver = {
        .name = "ec_csce2101_driver",
        .pm = &ec_pm_ops,
    },
    .probe = ec_probe,
    .remove = ec_remove,
    .id_table = ec_id_table,
};
module_i2c_driver(ec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("huangxionglue");
MODULE_DESCRIPTION("EC I2C Driver with Battery Monitoring and Event Handling");
