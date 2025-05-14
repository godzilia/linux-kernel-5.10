#include <linux/i2c.h>
#include <linux/module.h>

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

static int ec_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
    uint8_t ec_main_version;
    uint8_t ec_sub_version;
    uint8_t read_back;

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
    },
    .probe = ec_probe,
    .id_table = ec_id_table,
};
module_i2c_driver(ec_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("huangxionglue");
MODULE_DESCRIPTION("EC I2C Driver");
