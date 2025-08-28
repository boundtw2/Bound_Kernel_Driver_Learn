#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/fs.h>
#include <linux/cdev.h>

#include <linux/i2c.h>

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>


// 设备信息
#define MMA865X_DEVICE_NAME         "my_mma8653"
#define SENSOR_I2C_ADDR     0x1D
#define CLASS_NAME          "mma8653_class"

// MMA8653 寄存器定义
#define MMA865X_WHO_AM_I        0x0D
#define MMA865X_OUT_X_MSB       0x01
#define MMA865X_OUT_Y_MSB       0x03
#define MMA865X_OUT_Z_MSB       0x05
#define MMA865X_CTRL_REG1       0x2A
#define MMA865X_XYZ_DATA_CFG    0x0E

// 设备模式
#define MMA8653_MODE_STANDBY    0x00
#define MMA8653_MODE_ACTIVE     0x01

// 加速度数据结构
struct mma8653_data_t {
    short x;
    short y;
    short z;
};

// 驱动私有数据
struct simple_mma8653_dev {
    struct i2c_client *client;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    dev_t devno;
    int major;
};

static struct simple_mma8653_dev *mma8653_dev;


static int mma8653_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
    int ret = i2c_smbus_read_byte_data(client, reg);
    if (ret < 0) {
        dev_err(&client->dev, "读取寄存器 0x%02x 失败\n", reg);
        return ret;
    }
    *data = ret;
    return 0;
}

static int mma8653_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
    int ret = i2c_smbus_write_byte_data(client, reg, data);
    if (ret < 0) {
        dev_err(&client->dev, "写入寄存器 0x%02x 失败\n", reg);
    }
    return ret;
}

// 初始化传感器
static int mma8653_hw_init(struct i2c_client *client)
{
    u8 chip_id;
    int ret;

    // 读取芯片ID验证设备
    ret = mma8653_read_reg(client, MMA865X_WHO_AM_I, &chip_id);
    if (ret < 0) {
        return ret;
    }
    
    printk(KERN_INFO "MMA8653 芯片ID: 0x%02x\n", chip_id);

    // 设置为待机模式
    ret = mma8653_write_reg(client, MMA865X_CTRL_REG1, MMA8653_MODE_STANDBY);
    if (ret < 0) {
        return ret;
    }

    // 配置数据格式（±2g 量程）
    ret = mma8653_write_reg(client, MMA865X_XYZ_DATA_CFG, 0x00);
    if (ret < 0) {
        return ret;
    }

    // 设置为活动模式，50Hz 采样率
    ret = mma8653_write_reg(client, MMA865X_CTRL_REG1, 0x21); // Active + 50Hz
    if (ret < 0) {
        return ret;
    }

    msleep(100); // 等待传感器稳定

    printk(KERN_INFO "MMA8653 硬件初始化完成\n");
    return 0;
}


// 读取加速度数据
static int mma8653_read_accel_data(struct i2c_client *client, struct mma8653_data_t *data)
{
    u8 tmp_data[6];
    int ret;

    // 读取6个字节的加速度数据
    ret = i2c_smbus_read_i2c_block_data(client, MMA865X_OUT_X_MSB, 6, tmp_data);
    if (ret < 6) {
        dev_err(&client->dev, "读取加速度数据失败\n");
        return -EIO;
    }

    // 组合高低字节并转换为有符号数据
    data->x = ((tmp_data[0] << 8) | tmp_data[1]) >> 6;  // 10位数据右移6位
    data->y = ((tmp_data[2] << 8) | tmp_data[3]) >> 6;
    data->z = ((tmp_data[4] << 8) | tmp_data[5]) >> 6;

    // 处理符号位
    if (data->x & 0x200) data->x |= 0xFC00;  // 符号扩展
    if (data->y & 0x200) data->y |= 0xFC00;
    if (data->z & 0x200) data->z |= 0xFC00;

    return 0;
}


// 设备文件操作函数
static int mma8653_open(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "MMA8653 设备打开\n");
    filp->private_data = mma8653_dev;
    return 0;
}

static int mma8653_release(struct inode *inode, struct file *filp)
{
    printk(KERN_INFO "MMA8653 设备关闭\n");
    return 0;
}


static ssize_t mma8653_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct simple_mma8653_dev *dev = filp->private_data;
    struct mma8653_data_t accel_data;
    int ret;

    // 检查用户提供的缓冲区大小是否足够
    if (count < sizeof(struct mma8653_data_t)) {
        printk(KERN_WARNING "MMA8653: read buffer too small. Need %zu bytes.\n", sizeof(struct mma8653_data_t));
        return -EINVAL;
    }

    // 从传感器读取原始加速度数据
    ret = mma8653_read_accel_data(dev->client, &accel_data);
    if (ret < 0) {
        return ret; // 返回错误码
    }

    // 将包含原始short类型数据的结构体直接复制到用户空间
    if (copy_to_user(buf, &accel_data, sizeof(struct mma8653_data_t))) {
        printk(KERN_ERR "MMA8653: Failed to copy data to user space.\n");
        return -EFAULT;
    }

    // 返回成功复制的字节数
    return sizeof(struct mma8653_data_t);
}

static ssize_t mma8653_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    // 简单实现：写入任何数据都触发一次数据读取并打印
    struct simple_mma8653_dev *dev = filp->private_data;
    struct mma8653_data_t accel_data;
    int ret;

    ret = mma8653_read_accel_data(dev->client, &accel_data);
    if (ret == 0) {
        printk(KERN_INFO "当前加速度: X=%d, Y=%d, Z=%d\n", 
               accel_data.x, accel_data.y, accel_data.z);
    }

    return count;
}



// 文件操作结构体
static const struct file_operations mma8653_fops = {
    .owner = THIS_MODULE,
    .open = mma8653_open,
    .release = mma8653_release,
    .read = mma8653_read,
    .write = mma8653_write,
};




// I2C 设备ID表
static const struct i2c_device_id mma8653_i2c_id[] = {
    { MMA865X_DEVICE_NAME, 0 },
    { }
};
	
MODULE_DEVICE_TABLE(i2c, mma8653_i2c_id);

static int mma8653_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
    printk(KERN_INFO "MMA8653 I2C 设备探测开始\n");

	// 检查I2C功能
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C 适配器不支持所需功能\n");
        return -ENODEV;
    }

	// 分配设备结构体
    mma8653_dev = kzalloc(sizeof(struct simple_mma8653_dev), GFP_KERNEL);
    if (!mma8653_dev) {
        return -ENOMEM;
    }

    mma8653_dev->client = client;
    i2c_set_clientdata(client, mma8653_dev);

	// 初始化硬件
    ret = mma8653_hw_init(client);
    if (ret < 0) {
        dev_err(&client->dev, "硬件初始化失败\n");
        goto free_dev;
    }

	// 分配字符设备号
    ret = alloc_chrdev_region(&mma8653_dev->devno, 0, 1, MMA865X_DEVICE_NAME);
    if (ret < 0) {
        dev_err(&client->dev, "分配设备号失败\n");
        goto free_dev;
    }
    mma8653_dev->major = MAJOR(mma8653_dev->devno);

	// 初始化字符设备
    cdev_init(&mma8653_dev->cdev, &mma8653_fops);
    mma8653_dev->cdev.owner = THIS_MODULE;

	// 添加字符设备
    ret = cdev_add(&mma8653_dev->cdev, mma8653_dev->devno, 1);
    if (ret < 0) {
        dev_err(&client->dev, "添加字符设备失败\n");
        goto unregister_chrdev;
    }

	// 创建设备类
    mma8653_dev->class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(mma8653_dev->class)) {
        ret = PTR_ERR(mma8653_dev->class);
        dev_err(&client->dev, "创建设备类失败\n");
        goto del_cdev;
    }
	
	// 创建设备文件
    mma8653_dev->device = device_create(mma8653_dev->class, NULL, 
                                        mma8653_dev->devno, NULL, MMA865X_DEVICE_NAME);
    if (IS_ERR(mma8653_dev->device)) {
        ret = PTR_ERR(mma8653_dev->device);
        dev_err(&client->dev, "创建设备文件失败\n");
        goto destroy_class;
    }


    return 0;

destroy_class:
    class_destroy(mma8653_dev->class);
del_cdev:
    cdev_del(&mma8653_dev->cdev);
unregister_chrdev:
    unregister_chrdev_region(mma8653_dev->devno, 1);
free_dev:
    kfree(mma8653_dev);
    return ret;
}

// I2C 设备移除函数
static int mma8653_i2c_remove(struct i2c_client *client)
{
	struct simple_mma8653_dev *dev = i2c_get_clientdata(client);

    // 设置传感器为待机模式
    mma8653_write_reg(client, MMA865X_CTRL_REG1, MMA8653_MODE_STANDBY);

	// 清理设备文件和类
    device_destroy(dev->class, dev->devno);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devno, 1);
    kfree(dev);

    printk(KERN_INFO "MMA8653 驱动卸载完成\n");
    return 0;	

}


// I2C 驱动结构体
static struct i2c_driver mma8653_i2c_driver = {
    .driver = {
        .name = MMA865X_DEVICE_NAME,
        .owner = THIS_MODULE,
    },
    .probe = mma8653_i2c_probe,
    .remove = mma8653_i2c_remove,
    .id_table = mma8653_i2c_id,
};







static int __init mymma_init(void)
{
	int ret;
    
    printk(KERN_INFO "=== 开始加载简单 MMA8653 驱动 ===\n");
    printk(KERN_INFO "设备名称: %s\n", "my_mma8653");
    printk(KERN_INFO "设备地址: 0x%02x\n", SENSOR_I2C_ADDR);
    
    ret = i2c_add_driver(&mma8653_i2c_driver);
    if (ret < 0) {
        printk(KERN_ERR "注册 I2C 驱动失败，错误码: %d\n", ret);
        return ret;
    }
    
    printk(KERN_INFO "I2C 驱动注册成功\n");
    printk(KERN_INFO "等待设备匹配...\n");
    
    return 0;
	

}


static void __exit mymma_exit(void)
{
	printk(KERN_INFO "卸载简单 MMA8653 驱动\n");
    i2c_del_driver(&mma8653_i2c_driver);
	
}


module_init(mymma_init);
module_exit(mymma_exit);


MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code,  MMA8653 driver part");
MODULE_VERSION("1.0");
