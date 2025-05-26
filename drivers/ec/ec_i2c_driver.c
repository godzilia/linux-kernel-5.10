#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/syscore_ops.h>


#define EC_REG_START   					 (EC_MAIN_VERSION)   				 /* 起始寄存器地址 */
#define EC_REG_END     					 (EC_BAT_FULL_CHARGE_CAPACITY_H)     /* 结束寄存器地址 */
#define EC_MAX_RETRIES 					 (3)      							 /* 操作重试次数 */

#define EC_MAIN_VERSION  				 (0x00)								 
#define EC_SUB_VERSION   				 (0x01)
#define EC_SOC_ACPI_STATUS               (0x0A)
#define EC_POWER_STATUS  				 (0x20)
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


enum e_soc_acpi_status {
	ACPI_WORK 	  = 0x0,      //work  S0
	ACPI_S1 	  = 0x1,
	ACPI_S2 	  = 0x2,
	ACPI_STR 	  = 0x3,      //str
	ACPI_STD 	  = 0x4,      //std
	ACPI_POWEROFF = 0x5,      //poweroff
	ACPI_REBOOT   = 0x6,      //reboot
};

struct ec_device {
    struct i2c_client *client;
    struct regmap *regmap;
    struct mutex lock;
};

struct ec_device *ec;


/* EC寄存器读写状态回调 */
static bool ec_reg_volatile(struct device *dev, unsigned int reg)
{
    /* 某些寄存器可能是易变的，例如状态寄存器 */
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

static int ec_i2c_write(struct i2c_client *client, uint8_t reg, uint8_t val)
{
    int ret = -1;
    int retries = 0;
    uint8_t buf[2] = { reg, val };
    struct i2c_msg msg = {
        .flags = !I2C_M_RD,
        .addr = client->addr,
        .len = 2,
        .buf = buf,
    };

    while (retries < 5) {
        ret = i2c_transfer(client->adapter, &msg, 1);
        if (ret == 1)
            return 0;
        retries++;
    }

    return ret;
}

static int ec_i2c_read(struct i2c_client *client, uint8_t reg)
{
    int ret = -1;
    int retries = 0;
    uint8_t buf[2] = { reg, 0 };
    struct i2c_msg msgs[2] = {
        [0] = {
            .flags = 0,
            .addr = client->addr,
            .len = 1,
            .buf = &buf[0],
        },
        [1] = {
            .flags = I2C_M_RD,
            .addr = client->addr,
            .len = 1,
            .buf = &buf[1],
        }
    };

    while (retries < 5) {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if (ret == 2)
            return buf[1];
        retries++;
    }

    return ret;
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

/* 批量读取寄存器 */
static int ec_read_block(struct ec_device *ec, u8 reg, u8 *buf, int len)
{
    int ret;
    
    mutex_lock(&ec->lock);
    ret = regmap_bulk_read(ec->regmap, reg, buf, len);
    mutex_unlock(&ec->lock);
    
    return ret;
}

/* 批量写入寄存器 */
static int ec_write_block(struct ec_device *ec, u8 reg, const u8 *buf, int len)
{
    int ret;
    
    mutex_lock(&ec->lock);
    ret = regmap_bulk_write(ec->regmap, reg, buf, len);
    mutex_unlock(&ec->lock);
    
    return ret;
}
/* 设备挂起回调 */
static int ec_suspend(struct device *dev)
{
    struct ec_device *ec = dev_get_drvdata(dev);
    int ret = 0;
	u8 val = 0;

    dev_info(dev, "Suspending EC device...\n");
    
    /* 向EC发送挂起命令 */
    ret = ec_write_reg(ec, EC_SOC_ACPI_STATUS, ACPI_STR);
    if (ret) {
        dev_err(dev, "Failed to send suspend command: %d\n", ret);
        return ret;
    }
	ec_read_reg(ec,EC_SOC_ACPI_STATUS,&val);
	dev_info(dev, "%s:write ec EC_SOC_ACPI_STATUS: %d\n",__func__,val);

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
	ec_read_reg(ec,EC_SOC_ACPI_STATUS,&val);
    dev_info(dev, "%s:write ec EC_SOC_ACPI_STATUS: %d\n",__func__,val);
    return 0;
}
static SIMPLE_DEV_PM_OPS(ec_pm_ops, ec_suspend, ec_resume);

static void TH1520_syscore_shutdown(void)
{
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
		ec_read_reg(ec,EC_SOC_ACPI_STATUS,&val);
    	printk("%s:write ec EC_SOC_ACPI_STATUS: %d\n",__func__,val);
		mdelay(10);
#if 0
		while (1) {
			ret = ec_write_reg(ec, EC_SOC_ACPI_STATUS, ACPI_POWEROFF);
			if (ret) {
				mdelay(10);
				printk(" @@@@@@@@@@@@ fail power off %s %d \n",
					__func__, __LINE__);
			}
		}
#endif
	}else if(system_state == SYSTEM_RESTART) {
		printk("reboot %s\n",__func__);

		ret = ec_write_reg(ec, EC_SOC_ACPI_STATUS, ACPI_REBOOT);
		if(ret) {
			printk("set red-led error\n");
		}
		mdelay(10);
		ec_read_reg(ec,EC_SOC_ACPI_STATUS,&val);
    	printk( "%s:write ec EC_SOC_ACPI_STATUS: %d\n",__func__,val);
	}
}


static struct syscore_ops TH1520_syscore_ops = {
	.shutdown = TH1520_syscore_shutdown,
};

static int ec_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
	
    struct regmap_config config;

	uint8_t ec_main_version = 0;
    uint8_t ec_sub_version = 0;
	uint8_t read_back = 0;
    /* 分配并初始化设备结构 */
    ec = devm_kzalloc(&client->dev, sizeof(*ec), GFP_KERNEL);
    if (!ec)
        return -ENOMEM;

    ec->client = client;
    mutex_init(&ec->lock);
    i2c_set_clientdata(client, ec);

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
        return PTR_ERR(ec->regmap);
    }
	ec_read_reg(ec,0x00,&ec_main_version);
	ec_read_reg(ec,0x01,&ec_sub_version);
	printk(KERN_INFO "EC main read reagmap  version=0x%x, EC sub_version=0x%x\n", 
			   ec_main_version, ec_sub_version);
    dev_info(&client->dev, "EC device registered at 0x%02X\n", 
             client->addr);
    	


    // Read EC version
    ec_main_version = ec_i2c_read(client, 0x00);
    ec_sub_version = ec_i2c_read(client, 0x01);
    printk(KERN_INFO "EC main version=0x%x, EC sub_version=0x%x\n", 
           ec_main_version, ec_sub_version);

    // Write 0x55 to EC SRAM reg 0xaa
    if (ec_i2c_write(client, 0xaa, 0x55) != 0) {
        printk(KERN_ERR "Failed to write to EC register 0xaa\n");
        return -EIO;
    }

    read_back = ec_i2c_read(client, 0xaa);
    printk(KERN_INFO "0xaa=0x%x\n", read_back);
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
    .id_table = ec_id_table,
};
module_i2c_driver(ec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("huangxionglue");
MODULE_DESCRIPTION("EC I2C Driver");
