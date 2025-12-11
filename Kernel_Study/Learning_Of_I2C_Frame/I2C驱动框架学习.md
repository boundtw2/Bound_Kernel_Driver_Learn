# I2C驱动框架学习

0720 

查询相关资料，了解找到I2C驱动框架，定位到内核源码。通过书中初步介绍， tree /sys/bus/i2c/，发现内核中为 mma8653 加速度传感器（G-Sensor）注册 了I2C 设备信息。进一步回到书中学习I2C框架。

0721

继续学习书中内容，对书中的内容大概浏览了一番，并不完全理解。对四个重要结构体与相关概念进行了阅读。在源码中找到了mma8653的相关信息：i2c_board_info与注册代码。发现在driver/hwmon中有对mma8653驱动的实现，查看了一番，看不懂其中很多部分。对初始化部分进行了针对性浏览，纠结其动态获取i2c_adap的方式（动态配置注册），理解未果。发现内核代码中已对mma8653进行注册。

0722

学习driver/hwmon中的mma8653驱动代码，看不懂其使用input子系统等部分。通过ai编写了一个使用初级设备驱动的例程，阅读并尝试。尝试使用例程测试，在i2c_add_driver中似乎没有成功调用到probe函数进行初始化。查询后似乎在板子中已经运行了一个mma8653内核驱动的程序，并被编译到内核镜像中，无法移除。尝试解决未果。

0725

尝试重新编译内核代码，将内核源码板级文件中对mma8653的i2c_board_info中定义的type修改为my_mma8653，以图规避原驱动。尝试后成功，读取到了三轴加速度信息。 

0803

复习前面学到的内容，进一步理解四个数据结构各自的作用，了解在代码中如何拿到对应设备i2c_client的过程。

0804

深入阅读ai编写的代码，理解其代码中每一部分完成的工作以及其为什么这么做，调用了哪些函数完成了这些工作。

### 四个重要数据结构

#### i2c_adapter

i2c_adapter对应于一个物理适配器

```c
/*
 * i2c_adapter is the structure used to identify a physical i2c bus along
 * with the access algorithms necessary to access it.
 */
struct i2c_adapter {
	struct module *owner;
	unsigned int class;		  /* classes to allow probing for */
	const struct i2c_algorithm *algo; /* the algorithm to access the bus */
	void *algo_data;

	/* data fields that are valid for all devices	*/
	struct rt_mutex bus_lock;

	int timeout;			/* in jiffies */
	int retries;
	struct device dev;		/* the adapter device */

	int nr;
	char name[48];
	struct completion dev_released;

	struct mutex userspace_clients_lock;
	struct list_head userspace_clients;
};
```

#### i2c_algorithm

i2c_algorithm对应一套通信方法。

```c
struct i2c_algorithm {
	/* If an adapter algorithm can't do I2C-level access, set master_xfer
	   to NULL. If an adapter algorithm can do SMBus access, set
	   smbus_xfer. If set to NULL, the SMBus protocol is simulated
	   using common I2C messages */
	/* master_xfer should return the number of messages successfully
	   processed, or a negative value on error */
	int (*master_xfer)(struct i2c_adapter *adap, struct i2c_msg *msgs,
			   int num);
	int (*smbus_xfer) (struct i2c_adapter *adap, u16 addr,
			   unsigned short flags, char read_write,
			   u8 command, int size, union i2c_smbus_data *data);

	/* To determine what the adapter supports */
	u32 (*functionality) (struct i2c_adapter *);
};
```

​	关键函数master_xfer()用于产生C访问周期需要的信号，以i2c_msg（即 I2C 消息）为单位。i2c_msg结构体中的成员表明了I2C的传输地址、方向、缓冲区、缓冲区长度等信息。

```c
struct i2c_msg {
	__u16 addr;	/* slave address			*/
	__u16 flags;
#define I2C_M_TEN		0x0010	/* this is a ten bit chip address */
#define I2C_M_RD		0x0001	/* read data, from slave to master */
#define I2C_M_NOSTART		0x4000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_REV_DIR_ADDR	0x2000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_IGNORE_NAK	0x1000	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_NO_RD_ACK		0x0800	/* if I2C_FUNC_PROTOCOL_MANGLING */
#define I2C_M_RECV_LEN		0x0400	/* length will be first received byte */
	__u16 len;		/* msg length				*/
	__u8 *buf;		/* pointer to msg data			*/
};
```

#### i2c_driver

i2c_driver对应一套驱动方法。

```c
struct i2c_driver {
	unsigned int class;

	/* Notifies the driver that a new bus has appeared or is about to be
	 * removed. You should avoid using this, it will be removed in a
	 * near future.
	 */
	int (*attach_adapter)(struct i2c_adapter *) __deprecated;
	int (*detach_adapter)(struct i2c_adapter *) __deprecated;

	/* Standard driver model interfaces */
	int (*probe)(struct i2c_client *, const struct i2c_device_id *);
	int (*remove)(struct i2c_client *);

	/* driver model interfaces that don't relate to enumeration  */
	void (*shutdown)(struct i2c_client *);
	int (*suspend)(struct i2c_client *, pm_message_t mesg);
	int (*resume)(struct i2c_client *);

	/* Alert callback, for example for the SMBus alert protocol.
	 * The format and meaning of the data value depends on the protocol.
	 * For the SMBus alert protocol, there is a single bit of data passed
	 * as the alert response's low bit ("event flag").
	 */
	void (*alert)(struct i2c_client *, unsigned int data);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);

	struct device_driver driver;
	const struct i2c_device_id *id_table;

	/* Device detection callback for automatic device creation */
	int (*detect)(struct i2c_client *, struct i2c_board_info *);
	const unsigned short *address_list;
	struct list_head clients;
};
```

其主要成员函数是probe()、remove()、suspend()、resume()等，另外，struct i2c_device_id形式的id_table是该驱动所支持的I2C设备的ID表。

#### i2c_client 

i2c_client对应于真实的物理设备。每个I2C设备都需要一个i2c_client来描述。

```c
struct i2c_client {
	unsigned short flags;		/* div., see below		*/
	unsigned short addr;		/* chip address - NOTE: 7bit	*/
					/* addresses are stored in the	*/
					/* _LOWER_ 7 bits		*/
	char name[I2C_NAME_SIZE];
	struct i2c_adapter *adapter;	/* the adapter we sit on	*/
	struct i2c_driver *driver;	/* and our access routines	*/
	struct device dev;		/* the device structure		*/
	int irq;			/* irq issued by device		*/
	struct list_head detected;
};
```

#### 关系

##### （1）i2c_adapter与i2c_algorithm

​	一个I2C适配器需要i2c_algorithm提供的通信函数来控制适配器产生特定的访问周期。缺少i2c_algorithm的i2c_adapter什么也做不了，因此i2c_adapter中包含所使用的i2c_algorithm 的指针。

##### （2）i2c_driver与i2c_client

​	i2c_driver与i2c_client的关系是一对多，一个i2c_driver可以支持多个同类型的 i2c_client。

##### （3）i2c_adpater与i2c_client

​	i2c_adpater与i2c_client的关系与I2C硬件体系中适配器和设备的关系一致，即i2c_client依附于i2c_adpater。由于一个适配器可以连接多个I2C设备，所以一个i2c_adpater也可以被多个i2c_client依附，i2c_adpater中包括依附于它的i2c_client的链表。



### 代码学习

#### 源码

结合当前板子使用板级文件的方式与在内核中找到的代码，以及目前熟悉的初级字符设备驱动的方式，让ai编写了mma8653的驱动代码，代码如下：

```c
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

// 设备信息
#define DEVICE_NAME         "simple_mma8653"
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

// I2C 读写函数
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
    char output_buf[64];
    int len;
    int ret;

    if (count < sizeof(output_buf)) {
        return -EINVAL;
    }

    // 读取加速度数据
    ret = mma8653_read_accel_data(dev->client, &accel_data);
    if (ret < 0) {
        return ret;
    }

    // 格式化输出字符串
    len = snprintf(output_buf, sizeof(output_buf), 
                   "X: %4d, Y: %4d, Z: %4d\n", 
                   accel_data.x, accel_data.y, accel_data.z);

    // 复制到用户空间
    if (copy_to_user(buf, output_buf, len)) {
        return -EFAULT;
    }

    return len;
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

// I2C 设备探测函数
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
    ret = alloc_chrdev_region(&mma8653_dev->devno, 0, 1, DEVICE_NAME);
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
                                        mma8653_dev->devno, NULL, DEVICE_NAME);
    if (IS_ERR(mma8653_dev->device)) {
        ret = PTR_ERR(mma8653_dev->device);
        dev_err(&client->dev, "创建设备文件失败\n");
        goto destroy_class;
    }

    printk(KERN_INFO "MMA8653 字符设备驱动加载成功，主设备号: %d\n", mma8653_dev->major);
    printk(KERN_INFO "设备文件: /dev/%s\n", DEVICE_NAME);

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

// I2C 设备ID表
static const struct i2c_device_id mma8653_i2c_id[] = {
    { "my_mma8653", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mma8653_i2c_id);

// I2C 驱动结构体
static struct i2c_driver mma8653_i2c_driver = {
    .driver = {
        .name = "my_mma8653",
        .owner = THIS_MODULE,
    },
    .probe = mma8653_i2c_probe,
    .remove = mma8653_i2c_remove,
    .id_table = mma8653_i2c_id,
};

// 模块初始化
static int __init simple_mma8653_init(void)
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

// 模块退出
static void __exit simple_mma8653_exit(void)
{
    printk(KERN_INFO "卸载简单 MMA8653 驱动\n");
    i2c_del_driver(&mma8653_i2c_driver);
}

module_init(simple_mma8653_init);
module_exit(simple_mma8653_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Simple MMA8653 Character Device Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
```

#### 如何将i2c_client交到probe手里的？

1. 内核启动早期：
   `板级文件` → `i2c_register_board_info()`
   → 把 `(bus=1, addr=0x1D, name="my_mma8653")` 塞进全局链表。
2. I²C 适配器 omap_i2c.1 初始化：
   `i2c_register_adapter()` → `i2c_scan_static_board_info()`
   → 扫描链表 → `i2c_new_device()`
   → **i2c_client 实体诞生**，挂到总线。
3. insmod：
   `i2c_add_driver()` → `i2c_for_each_dev()`
   → 遍历总线 → `i2c_device_match()` 比对名字 → 匹配成功
   → `i2c_device_probe()` → **调你的 `mma8653_i2c_probe(client)`**
   → 你终于拿到 `client`，于是可以开始读 WHO_AM_I、CTRL_REG1 了。

#### i2c_set_clientdata有啥作用？

把驱动私有数据结构 `mma8653_dev` 的指针保存到 **I2C 设备 client** 的 `driver_data` 字段里。
这样在驱动的其它地方（如 remove、中断处理、open/read/write 等）就可以随时通过 `i2c_get_clientdata(client)` 把指针再取出来，从而拿到设备相关的所有信息（寄存器地址、缓冲区、互斥锁等）。
一句话：它建立了一条 **“client ↔ 驱动私有数据”** 的关联通道，后续代码无需再用全局变量，也能安全、快速地访问设备实例。

#### MODULE_DEVICE_TABLE有什么作用？

`MODULE_DEVICE_TABLE(i2c, mma8653_i2c_id);` 的主要作用是：

1. **自动化** - 支持设备的自动发现和驱动加载
2. **标准化** - 遵循Linux设备模型的标准做法
3. **便利性** - 简化设备管理，减少手动操作
4. **兼容性** - 支持设备树、热插拔等现代Linux特性

##### 创建模块别名信息

这个宏会在编译时生成模块别名信息，告诉内核模块系统这个驱动支持哪些设备：

```c
static const struct i2c_device_id mma8653_i2c_id[] = {
    { "my_mma8653", 0 },  // 设备名称和驱动数据
    { }                   // 数组结束标记
};
MODULE_DEVICE_TABLE(i2c, mma8653_i2c_id);
```

##### 生成的模块信息

编译后，可以通过以下命令查看生成的别名：

```bash
# 查看模块信息
modinfo your_driver.ko

# 输出示例：
# alias:          i2c:my_mma8653
# description:    Simple MMA8653 Character Device Driver
# author:         Your Name
# license:        GPL
```

#### dev_err() 有什么作用？

`dev_err()` 是Linux内核中的设备相关日志输出函数，它是内核日志系统的重要组成部分。

##### 1. 基本定义和作用

函数原型

```C
void dev_err(const struct device *dev, const char *fmt, ...)
```

参数说明

- `dev`: 指向设备结构体的指针，提供设备上下文信息
- `fmt`: 格式化字符串（类似printf）
- `...`: 可变参数列表

##### 2. 主要功能

设备上下文的错误日志

```C
// 您代码中的例子
dev_err(&client->dev, "I2C 适配器不支持所需功能\n");

// 输出格式类似：
// [时间戳] my_mma8653 1-001d: I2C 适配器不支持所需功能
```

自动添加设备信息

相比普通的 `printk(KERN_ERR, ...)`, `dev_err()` 会自动添加：

- 设备名称
- 总线信息
- 设备地址
- 驱动名称

##### 3. 日志级别系列函数

Linux内核提供了一套完整的设备日志函数：

```c
// 按严重程度排序（从高到低）
dev_emerg(dev, fmt, ...);   // 紧急情况
dev_alert(dev, fmt, ...);   // 警报
dev_crit(dev, fmt, ...);    // 严重错误
dev_err(dev, fmt, ...);     // 错误
dev_warn(dev, fmt, ...);    // 警告
dev_notice(dev, fmt, ...);  // 通知
dev_info(dev, fmt, ...);    // 信息
dev_dbg(dev, fmt, ...);     // 调试信息（默认不显示）
```

##### 4. 实际使用示例

在驱动代码中的应用

```
static int mma8653_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    // 检查I2C功能
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "I2C 适配器不支持所需功能\n");
        //        ↑              ↑
        //   设备对象指针    错误消息
        return -ENODEV;
    }

    // 其他错误处理示例
    ret = mma8653_hw_init(client);
    if (ret < 0) {
        dev_err(&client->dev, "硬件初始化失败，错误码: %d\n", ret);
        goto free_dev;
    }

    ret = alloc_chrdev_region(&mma8653_dev->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&client->dev, "分配设备号失败，错误码: %d\n", ret);
        goto free_dev;
    }
}
```

对比不同的日志函数

```c
// 1. 普通 printk（不推荐用于设备驱动）
printk(KERN_ERR "I2C 适配器不支持所需功能\n");
// 输出: [时间戳] I2C 适配器不支持所需功能

// 2. dev_err（推荐）
dev_err(&client->dev, "I2C 适配器不支持所需功能\n");
// 输出: [时间戳] my_mma8653 1-001d: I2C 适配器不支持所需功能

// 3. 其他级别示例
dev_info(&client->dev, "MMA8653 驱动探测成功，设备地址: 0x%02x\n", client->addr);
dev_warn(&client->dev, "传感器温度过高: %d°C\n", temp);
dev_dbg(&client->dev, "寄存器0x%02x值: 0x%02x\n", reg, value);
```

##### 5. 日志查看方法

通过dmesg查看

```bash
# 查看所有内核日志
dmesg

# 查看特定设备的日志
dmesg | grep "my_mma8653"

# 实时监控日志
dmesg -w

# 按时间排序查看
dmesg -T
```

通过journalctl查看（systemd系统）

```bash
# 查看内核消息
journalctl -k

# 查看特定设备消息
journalctl -k | grep "my_mma8653"

# 实时监控
journalctl -kf
```

##### 6. 调试配置

启用debug消息

```bash
# 临时启用某个驱动的debug消息
echo 'module my_mma8653 +p' > /sys/kernel/debug/dynamic_debug/control

# 或者在模块加载时启用
modprobe my_mma8653 dyndbg=+p
```

##### 7. 最佳实践

合理使用不同级别

```c
// 硬件故障或无法恢复的错误
dev_err(&client->dev, "芯片ID验证失败: 期望0x5A，实际0x%02x\n", chip_id);

// 可能影响功能但不是致命的问题  
dev_warn(&client->dev, "传感器校准值异常，使用默认值\n");

// 正常的状态信息
dev_info(&client->dev, "传感器初始化完成，量程: ±%dg\n", range);

// 详细的调试信息（发布版本中通常关闭）
dev_dbg(&client->dev, "寄存器dump: CTRL1=0x%02x, CTRL2=0x%02x\n", ctrl1, ctrl2);
```

错误消息编写原则

```c
// 好的错误消息：提供具体信息
dev_err(&client->dev, "I2C读取失败，寄存器0x%02x，错误码: %d\n", reg, ret);

// 不好的错误消息：信息不足
dev_err(&client->dev, "操作失败\n");
```

总结

`dev_err()` 的主要优势：

1. **上下文信息丰富** - 自动包含设备信息
2. **统一的日志格式** - 便于系统管理和调试
3. **分级管理** - 可以根据需要过滤不同级别的消息
4. **调试友好** - 便于定位问题到具体设备
5. **符合内核规范** - Linux内核开发的标准做法

#### 指定cdev.owner有什么意义？

1. `cdev.owner` 的类型

   ```c
   struct cdev {
       ...
       struct module *owner;
       ...
   };
   ```

   它是一个指向 `struct module` 的指针，用来记录“谁拥有这个字符设备”。

2. `THIS_MODULE` 是什么
   在模块源码里，预处理器把 `THIS_MODULE` 展开成指向当前编译单元所属的 `struct module` 对象的指针。
   例如你写的 `mma8653.ko` 里，`THIS_MODULE` 就是 `__this_module` 符号。

3. 绑定后会发生什么
   • 当你执行 `rmmod mma8653` 时，内核先检查该模块的引用计数（`module_refcount`）。
   • 只要 **cdev 还没被 `cdev_del()` 删除**，`owner` 指针就会使引用计数 > 0，于是 `rmmod` 会报错 `Resource temporarily unavailable`，从而阻止模块被卸载。
   • 一旦 `cdev_del()` 被调用，引用计数减 1，如果再也没有别的使用者，模块才真正允许卸载。

4. 不写这一行的后果
   如果漏掉 `owner = THIS_MODULE`，当用户空间程序仍打开着 `/dev/mma8653` 时，管理员执行 `rmmod mma8653` 会成功，接着内核把代码段/数据段从内存移除，随后 `read()/write()` 进入已卸载的地址空间，直接崩溃（oops 或 kernel panic）。

#### read/write的时候不直接调用全局变量mma8653_dev

1. **能不能？——能**
   你的驱动目前只支持一个 MMA8653，全局变量 `mma8653_dev` 在模块整个生命周期内都指向那块有效的内存，所以

   ```c
   static ssize_t mma8653_read(struct file *filp, char __user *buf, size_t cnt, loff_t *fpos)
   {
       struct simple_mma8653_dev *dev = mma8653_dev;   // 直接拿，也能跑
       ...
   }
   ```

   

   代码可以编译、可以运行、结果也正确。

2. **应不应该？——不应该，理由有三点**

   - **可移植性 / 未来扩展**
     一旦哪天硬件上挂了**两颗** MMA8653，内核会为每颗设备都调用一次 `probe`，届时就有**两份**独立的 `struct simple_mma8653_dev`，而全局指针只能保存其中一个。
     如果 read/write 里继续写死 `mma8653_dev`，第二颗设备的 `/dev/simple_mma8653-2` 打开后读到的永远是第一颗的数据，出现**交叉访问**。
     使用 `filp->private_data` 则天然支持“一个 fd 对应一份私有数据”，完全不受实例数量限制。
   - **API 约定**
     在字符设备框架里，`private_data` 就是留给驱动放“每次打开文件的私有上下文”的。
     你既然在 open 里把它填上了，read/write 就应当从那里取——这是 Linux 驱动里最普通、最可读的套路。
   - **测试 / 复用**
     按 `private_data` 写法的代码，**不用改一行**就能直接搬到“多实例”或“platform 设备 + 多个子节点”的场景。
     反之，全局变量的写法一旦扩展就得大改，还容易漏掉边角用例。

### 设备驱动相关

##### i2c_driver结构体

```c
struct i2c_driver {
	unsigned int class;

	/* Notifies the driver that a new bus has appeared or is about to be
	 * removed. You should avoid using this, it will be removed in a
	 * near future.
	 */
	int (*attach_adapter)(struct i2c_adapter *) __deprecated;
	int (*detach_adapter)(struct i2c_adapter *) __deprecated;

	/* Standard driver model interfaces */
	int (*probe)(struct i2c_client *, const struct i2c_device_id *);
	int (*remove)(struct i2c_client *);

	/* driver model interfaces that don't relate to enumeration  */
	void (*shutdown)(struct i2c_client *);
	int (*suspend)(struct i2c_client *, pm_message_t mesg);
	int (*resume)(struct i2c_client *);

	/* Alert callback, for example for the SMBus alert protocol.
	 * The format and meaning of the data value depends on the protocol.
	 * For the SMBus alert protocol, there is a single bit of data passed
	 * as the alert response's low bit ("event flag").
	 */
	void (*alert)(struct i2c_client *, unsigned int data);

	/* a ioctl like command that can be used to perform specific functions
	 * with the device.
	 */
	int (*command)(struct i2c_client *client, unsigned int cmd, void *arg);

	struct device_driver driver;
	const struct i2c_device_id *id_table;

	/* Device detection callback for automatic device creation */
	int (*detect)(struct i2c_client *, struct i2c_board_info *);
	const unsigned short *address_list;
	struct list_head clients;
};
```

1. ###### unsigned int class

   • 语义：把该驱动“归类”到某个 I²C 设备类别（I²C_CLASS_* 宏）。
   • 典型取值：

   - I²C_CLASS_HWMON       —— 硬件监测类（温度、电压、风扇）

   - I²C_CLASS_SPD         —— SPD/EE1004（内存条 EEPROM）

   - I²C_CLASS_DDC         —— 显示器 DDC/EDID

     • 作用：

   - 供 i2c-core 在总线遍历阶段做“快速过滤”。
     如果驱动 class 与 adapter class 不匹配，则根本不去 probe，节省启动时间。
     • 例：

   ```c
   static struct i2c_driver lm75_driver = {
       .class = I2C_CLASS_HWMON,
       ...
   };
   ```

2. ###### int (*attach_adapter)(struct i2c_adapter *)  __deprecated

   • 历史遗留接口，2.6 早期用于“手动遍历一条总线并创建设备”。
   • 现在已被 **device/driver-model + detect 回调** 取代，内核标记为废弃；新驱动**禁止**实现它。
   • 若仍使用，i2c-core 会打印 “deprecated” 警告。

3. ###### int (*detach_adapter)(struct i2c_adapter *)  __deprecated

   • attach_adapter 的“反操作”，同样废弃。

4. ###### int (*probe)(struct i2c_client *, const struct i2c_device_id *)

   • 设备与驱动“匹配成功”后被调用，做真正的初始化。
   • 形参：

- client：代表一个具体 I²C 设备（含 addr、adapter 指针、设备树节点等）。
- id：来自 id_table 的匹配项，常用于携带私有数据。
  • 返回值：0 表示成功，负 errno 表示失败，i2c-core 会回滚并继续尝试别的驱动。

5. ###### int (*remove)(struct i2c_client *)

   • 设备被卸载或驱动被 rmmod 时调用，负责释放 probe 中申请的资源。
   • 必须能正确处理“设备热插拔”与“驱动模块卸载”两种情况。

6. ###### void (*shutdown)(struct i2c_client *)

   • 系统关机 / reboot 流程（pm_power_off）时调用。
   • 典型场景：把 LED 驱动器设为安全状态、关闭电池充电等。

7. ###### int (*suspend)(struct i2c_client *, pm_message_t mesg)

   • 进入 suspend-to-RAM / suspend-to-disk 等低功耗模式前调用。
   • mesg.event 区分是 suspend、freeze 还是 poweroff。
   • 返回 0 表示可以挂起；负 errno 则阻止整个系统的 suspend。

8. ###### int (*resume)(struct i2c_client *)

   • 与 suspend 对称，唤醒后恢复设备运行状态。

9. ###### void (*alert)(struct i2c_client *, unsigned int data)

   • 仅当设备支持 SMBus Alert（或类似带中断线）时有用。
   • 发生 Alert 时，i2c-core 先通过“Alert Response Address”轮询，
   找到具体 client 后调用此回调。
   • data：SMBus 规范里只有 1 bit 事件标志，由硬件/协议决定。

10. ###### int (*command)(struct i2c_client *client, unsigned int cmd, void *arg)

    • 一个“私有的 ioctl”，驱动可自定义若干命令。
    • 用户空间可通过
    `ioctl(fd, I2C_SLAVE_COMMAND, &my_cmd)` 或
    `i2c_smbus_xfer()` 间接调用。
    • 多数驱动留空，改为用 regmap、hwmon、input 子系统提供的标准接口。

11. ###### struct device_driver driver

    • Linux 统一驱动模型的基类，包含 name、of_match_table、pm 等。
    • 必须初始化：

```c
.driver = {
    .name           = "my_chip",
    .of_match_table = of_match_ptr(my_of_match),
    .pm             = &my_pm_ops,
},
```

12. ###### const struct i2c_device_id *id_table

    • “传统 ID 匹配表”，用于非设备树场景。
    • 示例：

```c
static const struct i2c_device_id my_id[] = {
    { "my_chip", 0 },
    { "my_chip_a", 1 },
    { }
};
MODULE_DEVICE_TABLE(i2c, my_id);
```

• 当板级文件或平台代码里使用 `I2C_BOARD_INFO("my_chip", 0x50)` 时，
字符串“my_chip”与本表匹配，即可触发 probe。

13. ###### int (*detect)(struct i2c_client *, struct i2c_board_info *)

    • 自动探测未知设备（典型：hwmon、eeprom）。
    • 返回值：

- 0：确认检测到设备，填写 i2c_board_info，i2c-core 会自动创建设备。
- –ENODEV：未检测到。
  • 仅在同时提供 address_list 时生效。

14. ###### const unsigned short *address_list

    • detect 回调要扫描的地址列表，以 I2C_CLIENT_END 结尾。
    • 例：

```c
static const unsigned short addr_list[] = { 0x48, 0x49, 0x4a, I2C_CLIENT_END };
```

15. ###### struct list_head clients

    • **内部字段**，由 i2c-core 维护，用来链接本驱动当前已绑定的所有 client。
    • 驱动代码禁止直接操作，仅 i2c-core 使用。

------

小结：

- 新驱动只需实现 probe/remove，必要时补充 suspend/resume 和 driver.name/of_match_table。
- class、detect/address_list 仅在需要“自动探测”时填写。
- attach_adapter/detach_adapter 直接忽略。

#### 函数

这里汇集了所有Linux内核的I2C框架中的函数

##### 1. 驱动注册与注销

| 函数名                                      | 说明                                                         |
| :------------------------------------------ | :----------------------------------------------------------- |
| `i2c_add_driver(struct i2c_driver *driver)` | 注册一个 I2C 设备驱动到内核                                  |
| `i2c_del_driver(struct i2c_driver *driver)` | 注销一个 I2C 设备驱动                                        |
| `module_i2c_driver(driver)`                 | 宏，简化驱动注册与注销，自动调用 `i2c_add_driver` 和 `i2c_del_driver` |

------

#####  2. 设备探测与移除（probe/remove）

| 函数指针               | 说明                                   |
| :--------------------- | :------------------------------------- |
| `.probe = foo_probe`   | 当设备匹配成功时被调用，用于初始化设备 |
| `.remove = foo_remove` | 当设备被移除时被调用，用于清理资源     |

这两个函数是 `struct i2c_driver` 的核心成员，必须实现。

------

#####  3. 数据传输接口（I2C/SMBus）

| 函数名                                                       | 说明                                   |
| :----------------------------------------------------------- | :------------------------------------- |
| `i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)` | 通用 I2C 消息传输，支持多消息（读/写） |
| `i2c_master_send(struct i2c_client *client, const char *buf, int count)` | 向设备发送数据（单消息）               |
| `i2c_master_recv(struct i2c_client *client, char *buf, int count)` | 从设备接收数据（单消息）               |
| `i2c_smbus_read_byte_data(client, reg)`                      | 读取一个字节                           |
| `i2c_smbus_write_byte_data(client, reg, value)`              | 写一个字节                             |
| `i2c_smbus_read_i2c_block_data(...)`                         | 读取连续多个字节                       |
| `i2c_smbus_write_i2c_block_data(...)`                        | 写入连续多个字节                       |

> 这些函数都基于 `i2c_client` 操作，封装了底层 `i2c_adapter` 和 `i2c_algorithm` 的调用。

------

#####  4. 设备创建与销毁

| 函数名                                                       | 说明                                    |
| :----------------------------------------------------------- | :-------------------------------------- |
| `i2c_new_device(struct i2c_adapter *adap, struct i2c_board_info const *info)` | 手动创建一个 I2C 设备（client）         |
| `i2c_unregister_device(struct i2c_client *client)`           | 注销一个手动创建的设备                  |
| `i2c_new_dummy(struct i2c_adapter *adapter, u16 address)`    | 创建一个 dummy client（用于多地址设备） |

------

#####  5. 功能检查与辅助接口

| 函数名                                                       | 说明                              |
| :----------------------------------------------------------- | :-------------------------------- |
| `i2c_check_functionality(adapter, I2C_FUNC_I2C)`             | 检查适配器是否支持所需功能        |
| `i2c_get_clientdata(client)` / `i2c_set_clientdata(client, data)` | 获取/设置私有数据（替代全局变量） |
| `i2c_verify_client(struct device *dev)`                      | 确认设备是否为 I2C 客户端         |

------

#####  6. 设备匹配机制

| 机制             | 说明                     |
| :--------------- | :----------------------- |
| `id_table`       | 用于静态匹配设备名与地址 |
| `of_match_table` | 用于设备树（DTS）匹配    |
| `ACPI`           | 用于 ACPI 平台匹配       |

------

##### 7. 适配器注册（仅总线驱动开发时使用）

| 函数名                                         | 说明                              |
| :--------------------------------------------- | :-------------------------------- |
| `i2c_add_adapter(struct i2c_adapter *adapter)` | 注册一个新的 I2C 控制器（适配器） |
| `i2c_del_adapter(struct i2c_adapter *adapter)` | 注销适配器                        |

> 这些函数一般由 SoC 厂商提供，普通设备驱动开发者无需直接使用。



#### 1. 驱动注册与注销

##### 一、核心数据结构

| 结构体               | 作用                                                         |
| :------------------- | :----------------------------------------------------------- |
| `struct i2c_driver`  | 描述一个 I2C **设备驱动**（从设备驱动），包含 probe/remove 等回调 |
| `struct i2c_client`  | 描述一个 I2C **设备实例**（从设备），由内核在匹配成功后创建  |
| `struct i2c_adapter` | 描述一个 I2C **控制器**（主设备），由 SoC 厂商驱动注册       |

------

##### 二、注册接口

###### 1. 用户层入口（驱动开发者使用）

```c
#define i2c_add_driver(driver) \
    i2c_register_driver(THIS_MODULE, driver)
```

- 一般放在模块 `init` 函数中。
- 成功返回 0，失败返回负 errno。

###### 2. 内核层实现（源码流程）

```c
i2c_register_driver()
├── driver_register(&driver->driver);       // ① 把 driver->driver 挂到 bus(i2c_bus_type)->p->klist_drivers
│   └── 触发 bus 的 match / probe 流程
├── i2c_for_each_dev(driver, __process_new_driver);
    ├── __process_new_driver()
        ├── i2c_do_add_adapter()
            ├── i2c_detect()               // ② 遍历已注册 adapter，检测并创建 client
                └── i2c_detect_address()
                    └── device_register()  // 生成 i2c_client，并再次触发 match
                        └── driver->probe(client, id)  // ③ 最终调用
```

这里需要做出解释，此处设备驱动的probe应当是由driver_register 触发的。

1. i2c_register_driver()

- 先调用 driver_register(&driver->driver)
  - 这里走通用驱动框架（bus_add_driver → driver_attach → __driver_attach → really_probe → i2c_device_probe），对“已经存在并在 i2c_bus 上注册的 struct device（i2c_client）”做匹配并调用驱动的 probe/probe_new。在6818上，对应的i2c_client应在板级文件上已经完成注册。
  - 这一步不做地址扫描；它只处理“已经被枚举出来的设备”（比如来自设备树/ACPI/板级代码/i2c_new_client_device 等）。

2. i2c_for_each_dev(driver, __process_new_driver)

- “遍历已注册 adapter，检测并创建 client”的理解在语义上是对的（遍历适配器去做 legacy 探测），但具体函数名在不同内核版本上可能是 i2c_for_each_dev 或 i2c_for_each_adapter，源码注释常写“Walk the adapters that are already present”。关键不是名字，而是它会：
  - 仅当驱动实现了 .detect 并提供 .address_list/.class 时，才对每个已存在的 adapter 调用 i2c_do_add_adapter，进而 i2c_detect。
  - i2c_detect 内部按 address_list 对总线发探测，若确认有设备，就创建一个新的 i2c_client（路径里你写到 i2c_detect_address → device_register，这在具体实现中可能是 i2c_new_device/device_add，语义一致）。
  - 新的 i2c_client 注册到驱动框架后，走一遍匹配/attach/probe（即你标注的“再次触发 match → 最终调用 driver->probe”）。

这里并不会出现“同一设备被 probe 两次”

- device 层面的单绑定：每个 struct device 只能绑定一个 dev->driver；已经绑定的设备，在后续匹配中会被跳过（__driver_attach/device_attach 会检查）。
- 地址冲突检查：在通过 detect/new_device 创建 i2c_client 前，核心会检查该适配器上的地址是否已被占用（如 i2c_check_addr_busy），已存在就不会重复创建 client，自然也不会触发第二次 probe。
- 并发与时序下的幂等：即使在注册期间发生并发添加/匹配，最终也只能成功绑定一次；第二次尝试会因为 dev->driver 已非空而失败/跳过。

因此，更精准的表述是：

- ① driver_register 负责给“已注册的 i2c_client”触发 probe；
- ②→③ i2c_for_each_dev（或同义的适配器遍历）只在驱动支持 legacy 探测时，扫描“已注册的适配器”，发现设备后“创建新的 i2c_client”，再触发该“新设备”的 probe；
- 同一设备不会被重复绑定，适配器上同一地址不会重复创建 client，所以不会出现实质的“双 probe 同一设备”。

而现代内核/驱动多采用 DT/ACPI 等显式枚举，通常不实现 .detect，此时 i2c_for_each_dev 这条链路基本是空操作，不会创建新 client、也不会引发额外的 probe。

------



##### 三、注销接口

###### 1. 用户层入口

```c
void i2c_del_driver(struct i2c_driver *driver);
```

- 一般放在模块 `exit` 函数中。

###### 2. 内核层实现

```c
i2c_del_driver()
├── driver_unregister(&driver->driver);  // 从总线链表摘除
│   └── 触发 bus 的 remove 流程
│       └── driver->remove(client);      // 逐个对匹配到的 client 调用
├── 释放 driver 占用的内部资源
```

> 关键点：
>
> - 注销时会**自动遍历**所有与该 driver 绑定的 client，依次执行 remove；
> - 开发者只需在 remove 中释放 probe 里分配的资源，无需手动遍历 client。

------

##### 四、典型代码模板

```c
static const struct i2c_device_id my_ids[] = {
    { "my_chip", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, my_ids);

static struct i2c_driver my_driver = {
    .driver = {
        .name           = "my_chip",
        .of_match_table = of_match_ptr(my_of_ids), // 可选
    },
    .probe  = my_probe,
    .remove = my_remove,
    .id_table = my_ids,
};

static int __init my_init(void)
{
    return i2c_add_driver(&my_driver);
}

static void __exit my_exit(void)
{
    i2c_del_driver(&my_driver);
}

module_init(my_init);
module_exit(my_exit);
MODULE_LICENSE("GPL");
```

ps：`MODULE_DEVICE_TABLE` 是 Linux 内核模块中的一个**宏**，它的核心作用是：

> **把“本模块支持哪些设备”的信息以只读形式静态地编译进 ko 文件，使得内核或用户空间工具（如 `modprobe`、`udev`）可以在**不加载模块**的情况下就知道“这个模块能为哪些设备提供服务”**。

------

##### 五、常见问题与调试技巧

| 现象              | 排查点                                                       |
| :---------------- | :----------------------------------------------------------- |
| probe 未被调用    | 1. 设备树/ACPI 未正确描述；2. 地址不匹配；3. adapter 尚未注册 |
| remove 未被调用   | 确保模块使用 `i2c_del_driver` 注销，而不是直接 `rmmod` 导致引用泄漏 |
| 探测到多个 client | 防止 `id_table` 与 `of_match_table` 同时命中，导致重复探测   |

------

##### 六、小结

| 阶段   | 函数/流程                                    | 说明                                         |
| :----- | :------------------------------------------- | :------------------------------------------- |
| 注册   | `i2c_add_driver()` → `i2c_register_driver()` | 把驱动挂到总线，**立即尝试**匹配所有 adapter |
| 匹配   | `i2c_detect()` → `device_register()`         | 生成 `i2c_client` 并再次 match               |
| 初始化 | `driver->probe()`                            | 驱动真正干活的地方                           |
| 注销   | `i2c_del_driver()` → `driver_unregister()`   | 自动解除所有绑定并调用 remove                |



#### 2. 设备探测与移除

##### 一、机制原理：总线-设备-驱动模型

1. 总线类型
   `struct bus_type i2c_bus_type` 在 `drivers/i2c/i2c-core-base.c` 中定义为

   ```c
   .match  = i2c_device_match,   // 匹配规则
   .probe  = i2c_device_probe,   // 匹配成功后的统一“探测”入口
   .remove = i2c_device_remove,  // 设备或驱动被卸载时调用
   ```

   所有 I²C 设备/驱动都会挂在这条总线上，生命周期由内核统一调度。

2. 匹配规则（match）
   a) 如果设备由 **设备树** 描述，`of_match_table` 中的 `compatible` 参与匹配；
   b) 如果由 **ACPI** 描述，`_HID/_CID` 参与匹配；
   c) 传统方式下，`id_table` 中的名字与 `i2c_board_info.type` 匹配；
   d) 若驱动实现了 `detect()` 回调，也可在运行时扫描地址并动态创建设备。

一旦匹配成功，`i2c_device_probe` 被调，最终走到驱动自己填写的 `xxx_probe()`。

------

##### 二、内核源码级流程

1. 探测（probe）

```
用户层：i2c_add_driver()
→ i2c_register_driver()
   → driver_register(&driver->driver)   // 把驱动挂到 i2c_bus_type
       → bus_for_each_dev()             // 对现有 adapter 上的所有 client
           → __device_attach()
               → i2c_device_probe()
                   → driver->probe(client, id)  // 驱动实现
```

此时驱动拿到一个 **已经创建好的** `struct i2c_client`，完成：

- 读取 WHO_AM_I 验证芯片 ID
- 初始化寄存器、申请 GPIO/中断、注册 input 子系统或字符设备等
- 用 `i2c_set_clientdata()` 把私有结构挂到 `client->dev.driver_data` 上。

2. 移除（remove）

```
用户层：i2c_del_driver()
→ driver_unregister()
   → bus_remove_driver()
       → device_release_driver()
           → i2c_device_remove()
               → driver->remove(client)  // 驱动实现
```

驱动的 `xxx_remove()` 应该做：

- 关闭传感器、把寄存器置为 standby
- 释放中断、gpio、内存、字符设备号、sysfs 节点
- `i2c_set_clientdata(client, NULL)` 清指针，防止 use-after-free。

------

##### 三、驱动接口清单

| 阶段             | 驱动需要实现的成员                                          | 内核提供的辅助 API                                        |
| :--------------- | :---------------------------------------------------------- | :-------------------------------------------------------- |
| 匹配             | `.id_table`、`.of_match_table`                              | `i2c_match_id()`                                          |
| 探测             | `.probe(struct i2c_client *, const struct i2c_device_id *)` | `i2c_check_functionality()`、`i2c_smbus_read_byte_data()` |
| 移除             | `.remove(struct i2c_client *)`                              | `i2c_set_clientdata()`、`i2c_get_clientdata()`            |
| 动态检测（可选） | `.detect()`、`.address_list`                                | `i2c_new_device()`                                        |

------

##### 四、调试与常见问题

1. **probe 不执行**
   - 检查设备树 compatible 与驱动 `of_match_table` 是否一字不差；
   - 确认 adapter 已注册（`i2cdetect -l`）；
   - 地址是否冲突，总线是否被占用（`i2cdetect -y 1`）。
2. **remove 不调用 / 模块卸载死机**
   - 是否忘记在 remove 里释放资源，导致引用计数非 0；
   - 是否仍有进程打开设备节点导致 `module_refcount` 非 0。
3. **热插拔场景**
   - 对可热插拔 I²C 设备（如 HDMI 转接板），必须保证 detect/probe 可重入，remove 时彻底回滚。

------

##### 五、一张图总结（文字版）

```
+---------------+        +------------------+        +------------------+
| i2c_adapter   |   1    |  i2c_client      |   2    |  i2c_driver      |
| (I2C 控制器)   |------->| (代表一个从设备)  |<------>| (你的驱动代码)    |
|               |        |                  |        | .probe  .remove  |
+---------------+        +------------------+        +------------------+
```

1. **注册 adapter**（由 SoC 厂商完成）
2. **注册 driver** → 内核 `match` → `probe` → 业务逻辑
3. **设备消失或 rmmod** → `remove` → 资源回收

通过以上四步闭环，Linux I²C 子系统实现了“驱动-设备”自动绑定、即插即用。

#### 3.  数据传输接口（I2C/SMBus）

##### 1. i2c_transfer – 万用“发动机”

• 功能
一次下发 **任意条** `i2c_msg`，能拼出 **所有** I²C 序列：
写-重启-读、10-bit 地址、无停止位重启、复合报文等。

• 原型

```c
int i2c_transfer(struct i2c_adapter *adap,
                 struct i2c_msg      msgs[],
                 int                 num);
```

成功返回已处理的 msg 条数；失败返回负 errno。

• 底层实现
		拿总线锁 → 调用适配器实现的 `algo->master_xfer()` → 释放锁。

```
用户驱动：i2c_transfer()
        │
        ├─ DEBUG 打印
        ├─ 锁：trylock / lock
        ├─ for (retry)
        │     └── adap->algo->master_xfer()  // 硬件干活
        ├─ 解锁
        └─ 返回（成功条数 / -errno）
```

• 典型序列 – “先写寄存器地址再读 2 字节”

```c
struct i2c_msg msg[2];
u8 reg = 0x01;
u8 buf[2];

msg[0].addr  = client->addr;
msg[0].flags = 0;               /* 写 */
msg[0].len   = 1;
msg[0].buf   = &reg;

msg[1].addr  = client->addr;
msg[1].flags = I2C_M_RD;        /* 读 */
msg[1].len   = 2;
msg[1].buf   = buf;

ret = i2c_transfer(client->adapter, msg, 2);
```

• 陷阱

- 返回 `num` 表示 **全部** msg 成功；返回 0~num-1 表示中途失败。
- **不能**在 atomic 上下文调用（可能睡眠）。



##### 2. i2c_master_send – 单条写


• 原型

```c
int i2c_master_send(struct i2c_client *client,
                    const char *buf,
                    int count);
```

等价于

```c
struct i2c_msg msg = {
    .addr  = client->addr,
    .flags = 0,
    .len   = count,
    .buf   = (char *)buf,
};
return i2c_transfer(client->adapter, &msg, 1);
```

• 典型用法

```c
u8 data[3] = {0x20, 0x0F, 0x08};
i2c_master_send(client, data, 3);
```

• 陷阱
长度超过 32/64 Byte 时某些适配器会 `-EIO`，需改用 `i2c_transfer` 多条。



##### 3. i2c_master_recv – 单条读


• 原型

```c
int i2c_master_recv(struct i2c_client *client,
                    char *buf,
                    int count);
```

等价于

```c
struct i2c_msg msg = {
    .addr  = client->addr,
    .flags = I2C_M_RD,
    .len   = count,
    .buf   = buf,
};
return i2c_transfer(client->adapter, &msg, 1);
```



##### 4. i2c_smbus_read_byte_data – 读 8-bit 寄存器

• 原型

```c
s32 i2c_smbus_read_byte_data(struct i2c_client *client, u8 reg);
```

返回 0x00~0xFF；错误返回负 errno。

• 底层

```c
i2c_smbus_xfer(adapter, client->addr, client->flags,
               I2C_SMBUS_READ, reg,
               I2C_SMBUS_BYTE_DATA, &data);
```

构造两条 msg：
① 写寄存器地址 → ② 读 1 Byte。



##### 5. i2c_smbus_write_byte_data – 写 8-bit 寄存器

• 原型

```c
s32 i2c_smbus_write_byte_data(struct i2c_client *client,
                              u8 reg, u8 value);
```

成功返回 0；失败负 errno。

• 底层
一条写 msg：
start + addr + reg + value + stop。



##### 6/7.  i2c_smbus_read/write_i2c_block_data – 块读写

• 原型

```c
s32 i2c_smbus_read_i2c_block_data(struct i2c_client *client,
                                  u8 reg, u8 length, u8 *values);

s32 i2c_smbus_write_i2c_block_data(struct i2c_client *client,
                                   u8 reg, u8 length, const u8 *values);
```

• 限制

- 最多 32 Byte（SMBus 2.0 规范）；
- 某些适配器需要 `I2C_FUNC_SMBUS_I2C_BLOCK` 能力。
- tips：**只能“连续读”那些** *本身支持寄存器地址自动递增* **的器件**

• 底层
读：先写 reg → 再读 length 字节；
写：先写 reg → 再写 length 字节。

• 示例 – 读取 6 轴加速度 6 字节

```c
u8 buf[6];
i2c_smbus_read_i2c_block_data(client, OUT_X_MSB, 6, buf);
```



##### 汇总对比表 

| 函数                             | 使用场景     | 最大长度     | 可否多条消息  | 是否在总线加锁 |
| :------------------------------- | :----------- | :----------- | :------------ | :------------- |
| `i2c_transfer`                   | 任意序列     | 受适配器限制 | ✅多条         | ✅              |
| `i2c_master_send`                | 单写         | 受适配器限制 | ❌单条         | ✅              |
| `i2c_master_recv`                | 单读         | 受适配器限制 | ❌单条         | ✅              |
| `i2c_smbus_read_byte_data`       | 读寄存器 1B  | 1B           | ❌(内部已拼好) | ✅              |
| `i2c_smbus_write_byte_data`      | 写寄存器 1B  | 1B           | ❌             | ✅              |
| `i2c_smbus_read_i2c_block_data`  | 读寄存器 N B | ≤32B         | ❌(内部已拼好) | ✅              |
| `i2c_smbus_write_i2c_block_data` | 写寄存器 N B | ≤32B         | ❌             | ✅              |

#### 4. I2C的设备创建与销毁

##### 一、两条诞生/消亡路径

| 路径                                    | 描述方式               | 创建者                                       | 销毁触发                  | 典型场景               |
| :-------------------------------------- | :--------------------- | :------------------------------------------- | :------------------------ | :--------------------- |
| 1. 静态板级 (`i2c_register_board_info`) | 内核或启动代码提前声明 | `i2c_register_board_info` → `i2c_new_device` | 模块卸载 / 设备树节点消失 | 老 ARM 平台、x86 BIOS  |
| 2. 动态探测 (`detect`)                  | 驱动运行期发现         | `i2c_new_device` / `i2c_new_probed_device`   | 驱动移除 / 设备消失       | 热插拔转接板、自动探测 |

------

##### 二、三类核心 API

1. **创建设备**

   ```c
   struct i2c_client *
   i2c_new_device(struct i2c_adapter *adap,
                  const struct i2c_board_info *info);
   ```

   - **同步**完成：立即生成 `i2c_client` 并尝试匹配驱动。
   - `info->addr` 必须事先确认在总线上存在，否则只是“空壳”设备。

2. **自动探测并创建设备**

   ```c
   struct i2c_client *
   i2c_new_probed_device(struct i2c_adapter *adap,
                         struct i2c_board_info *info,
                         unsigned short const *addr_list,
                         int (*probe)(struct i2c_adapter *, unsigned short));
   ```

   

   - 遍历 `addr_list`，对每个地址调用 `probe()`，成功后创建 client。
   - 常用于 **未知地址** 的外设（如 EEPROM 在 0x50~0x57 之间）。

3. **注销设备**

   ```c
   void i2c_unregister_device(struct i2c_client *client);
   ```

   - **同步**销毁：立即调用 `device_unregister()` → 触发 `remove()`。

------

##### 三、四个关键对象

| 对象             | 作用                   | 生命周期    | 与驱动关系           |
| :--------------- | :--------------------- | :---------- | :------------------- |
| `i2c_adapter`    | 一条总线控制器         | 与 SoC 同寿 | 由 SoC 驱动注册      |
| `i2c_board_info` | 设备“蓝图”             | 仅一次创建  | 静态/动态填充        |
| `i2c_client`     | 总线上“一个从设备实例” | 创建 ↔ 注销 | 与驱动 1:N           |
| `device_node`    | 设备树节点             | 与 DTB 同寿 | 通过 `of_match` 关联 |

##### 四、代码级最小示例

```c
/* 1. 静态板级：老平台 */
static struct i2c_board_info my_board[] __initdata = {
    { I2C_BOARD_INFO("mma8653", 0x1D), },
};
i2c_register_board_info(1, my_board, ARRAY_SIZE(my_board));

/* 2. 动态探测：未知地址 */
static const unsigned short addr_list[] = { 0x50, 0x51, 0x52, I2C_CLIENT_END };
struct i2c_board_info info = { .type = "eeprom" };
client = i2c_new_probed_device(adap, &info, addr_list, NULL);
...
/* 3. 销毁 */
i2c_unregister_device(client);
```

------

##### 五、常见问题速查

| 现象                       | 根因                         | 解决                            |
| :------------------------- | :--------------------------- | :------------------------------ |
| `i2c_new_device` 返回 NULL | adapter 未注册 / 地址冲突    | 先确认 `i2cdetect` 能看到地址   |
| 设备树节点 probe 不执行    | compatible 不匹配            | 检查 `of_match_table`           |
| 模块卸载后 client 仍存在   | 忘记 `i2c_unregister_device` | 在 remove 中显式注销            |
| 热插拔转接板设备丢失       | detect 失败                  | 在驱动里实现 detect + addr_list |

------

一句话总结

- **创建设备**：`i2c_new_device()`/`i2c_new_probed_device()` 把“蓝图”变成 `i2c_client`，内核立即匹配并执行 probe。
- **销毁设备**：`i2c_unregister_device()` 把 `i2c_client` 从总线摘下来，触发驱动 remove，完成资源回滚。



#### 5. 功能检查与辅助接口

##### 一、功能检查（Capability Check）  

| API 原型                                  | 检查内容                              | 典型返回值              | 何时调用                | 使用示例                                                     |
| ----------------------------------------- | ------------------------------------- | ----------------------- | ----------------------- | ------------------------------------------------------------ |
| `i2c_check_functionality(adapter, flags)` | 总线 **控制器** 是否具备 **位级功能** | 0：不支持；非 0：支持   | probe 里 **第一条语句** | `if (!i2c_check_functionality(adap, I2C_FUNC_I2C)) return -ENODEV;` |
| `i2c_check_quirks(adapter, quirks)`       | 控制器是否有 **硬件缺陷/限制**        | 0：无限制；非 0：有限制 | probe 里 **第二条语句** | `if (i2c_check_quirks(adap, I2C_AQ_NO_ZERO_LEN)) dev_warn(...);` |

###### 1.1 功能掩码（flags）  
- **位掩码** 定义在 `include/uapi/linux/i2c.h`，常用：  
  
  - `I2C_FUNC_I2C` —— 支持原始 I²C 字节流  
  - `I2C_FUNC_10BIT_ADDR` —— 支持 10-bit 地址  
  - `I2C_FUNC_SMBUS_BYTE` / `WORD` / `BLOCK` —— SMBus 子协议  
  - `I2C_FUNC_NOSTART` —— 支持 **重复起始但不发 STOP**（少见）  
- 检查规则：**位与** 非 0 即支持  
  ```c
  if (!(i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA)))
        return -ENOTSUPP;
  ```

###### 1.2 缺陷掩码（quirks）  
- `I2C_AQ_NO_ZERO_LEN` —— 控制器 **不能发送 0 字节消息**  
- `I2C_AQ_NO_REPEATED_START` —— 不支持重复起始  
- `I2C_AQ_COMB_WRITE_THEN_READ` —— 只支持 **先写后读** 复合消息  
- 通过 **adapter quirks** 位图检查，驱动可按需 **切回 PIO / 分段传输**。

------------------------------------------------
##### 二、辅助接口（Helper API）  

| 功能                   | 接口                                                         | 说明                                       | 示例                                                    |
| ---------------------- | ------------------------------------------------------------ | ------------------------------------------ | ------------------------------------------------------- |
| **私有数据**           | `i2c_set_clientdata(client, data)` / `i2c_get_clientdata(client)` | 取代全局指针，支持 **多实例**              | `i2c_set_clientdata(client, my_data);`                  |
| **校验设备**           | `i2c_verify_client(struct device *dev)`                      | 判断 `struct device` 是否为合法 I²C 客户端 | `if (!i2c_verify_client(dev)) return -EINVAL;`          |
| **锁总线**             | `i2c_lock_adapter(adapter)` / `i2c_unlock_adapter(adapter)`  | 手动加/解锁 **bus_lock** mutex             | 自定义复合传输                                          |
| **trylock**            | `i2c_trylock_adapter(adapter)`                               | 非阻塞拿锁，失败返回 0                     | 中断上下文快速检测                                      |
| **遍历总线**           | `i2c_for_each_dev(void *data, int (*fn)(struct device *, void *))` | 对 **已挂载的 client** 执行回调            | 调试器、热插拔守护                                      |
| **获取 adapter**       | `i2c_get_adapter(int nr)` / `i2c_put_adapter(struct i2c_adapter *adap)` | 通过编号引用计数拿适配器                   | 用户空间工具 `i2c-tools`                                |
| **注册字符设备快捷宏** | `I2C_CLIENT_END`                                             | 地址列表结束标记                           | `const u16 addr_list[] = {0x50, 0x51, I2C_CLIENT_END};` |

------------------------------------------------
##### 三、完整代码片段：probe 里“查 + 用”  
```c
static int foo_probe(struct i2c_client *client)
{
    struct foo_data *data;
    struct i2c_adapter *adap = client->adapter;

    /* 1. 能力检查 */
    if (!i2c_check_functionality(adap, I2C_FUNC_I2C |
                                       I2C_FUNC_SMBUS_BYTE_DATA))
        return -ENOTSUPP;

    /* 2. 缺陷检查 */
    if (i2c_check_quirks(adap, I2C_AQ_NO_ZERO_LEN)) {
        dev_info(&client->dev,
                 "controller cannot do zero-length msg, fallback\n");
    }

    /* 3. 私有数据 */
    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;
    i2c_set_clientdata(client, data);

    /* 4. 读取芯片 ID 验证 */
    return 0;
}
```

------------------------------------------------
##### 四、调试技巧  
1. **打印能力位**  
   ```c
   dev_dbg(&adap->dev, "capabilities: 0x%lx\n",
           adap->algo->functionality(adap));
   ```
2. **动态查看 quirks**  
   `cat /sys/class/i2c-adapter/i2c-0/quirks`（若驱动导出）  
3. **用户空间检测**  
   `i2cdetect -F 0` 会列出 `/dev/i2c-0` 支持的功能位。

------------------------------------------------
一句话总结  
- **功能检查**：`i2c_check_functionality()` 保证 **控制器** 能做你打算做的事；  
- **缺陷检查**：`i2c_check_quirks()` 提前规避 **硬件怪癖**；  
- **辅助接口**：锁、私有数据、适配器引用计数，是写健壮、多实例驱动的“瑞士军刀”。



#### 6. 设备匹配机制

下面把 Linux I²C 子系统的 **设备匹配机制** 彻底拆成 **三条主线、四个阶段、五种数据结构、六段代码路径**，读完即可在任意平台（ARM/ACPI/x86）写出“一次编写、到处匹配”的驱动。

----------------------------------------------------------------
##### 一、三条主线：id_table、of_match_table、ACPI  
| 主线                | 描述方式              | 存放位置 | 匹配键                   | 典型文件           |
| ------------------- | --------------------- | -------- | ------------------------ | ------------------ |
| 1. `id_table`       | 数组 `i2c_device_id`  | 驱动源码 | `.name` + `.driver_data` | `mydrv.c`          |
| 2. `of_match_table` | 数组 `of_device_id`   | 驱动源码 | `.compatible`            | `mydrv.c` / `.dts` |
| 3. ACPI             | 数组 `acpi_device_id` | 驱动源码 | `_HID` / `_CID`          | `mydrv.c` / `DSDT` |

> 注：三条线可同时启用，**优先级**（从高到低）  
> Device Tree > ACPI > id_table

----------------------------------------------------------------
##### 二、四个匹配阶段（内核源码视角）  
1. **设备注册阶段**  
   - 静态板级：  
     `i2c_register_board_info()` → `i2c_new_device()`  
   - 设备树：  
     `of_i2c_register_devices()` → `i2c_new_device()`  
   - ACPI：  
     `acpi_i2c_register_devices()` → `i2c_new_device()`  

2. **client 创建**  
   每个 `i2c_client` 记录 **addr、type、of_node、acpi_companion**。

3. **match 回调**  
   核心函数：`i2c_device_match(struct device *dev, struct device_driver *drv)`  
   执行顺序：  
   ```
   of_driver_match_device()   // ① Device Tree
   acpi_driver_match_device() // ② ACPI
   i2c_match_id()             // ③ id_table
   ```

4. **probe 调用**  
   匹配成功 → `__device_attach()` → `driver->probe(client, id)`。

----------------------------------------------------------------
##### 三、五种关键数据结构  
| 结构体                  | 所在位置              | 作用                          |
| ----------------------- | --------------------- | ----------------------------- |
| `struct i2c_device_id`  | `<linux/i2c.h>`       | 传统 id_table 条目            |
| `struct of_device_id`   | `<linux/of_device.h>` | 设备树 compatible             |
| `struct acpi_device_id` | `<linux/acpi.h>`      | ACPI _HID/_CID                |
| `struct i2c_client`     | `<linux/i2c.h>`       | 运行时设备实例                |
| `struct i2c_driver`     | 驱动定义              | 包含上述三张表 + probe/remove |

----------------------------------------------------------------
##### 四、id_table 静态匹配（示例）  
```c
static const struct i2c_device_id mma8653_id[] = {
    { "mma8653", 0x8653 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mma8653_id);

static struct i2c_driver mma8653_driver = {
    .driver = {
        .name = "mma8653",
    },
    .id_table = mma8653_id,
    .probe    = mma8653_probe,
};
```
- 板级代码：  
  ```c
  static struct i2c_board_info i2c_devs1[] __initdata = {
      { I2C_BOARD_INFO("mma8653", 0x1D), },
  };
  i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));
  ```
- 匹配规则：  
  `client->name` 与 `id_table[i].name` 字符串 **完全相等**。

----------------------------------------------------------------
##### 五、of_match_table 设备树匹配（示例）  
```c
static const struct of_device_id mma8653_of_match[] = {
    { .compatible = "nxp,mma8653" },
    { }
};
MODULE_DEVICE_TABLE(of, mma8653_of_match);

static struct i2c_driver mma8653_driver = {
    .driver = {
        .of_match_table = mma8653_of_match,
    },
};
```
- 设备树：  
  ```dts
  &i2c1 {
      accelerometer@1d {
          compatible = "nxp,mma8653";
          reg = <0x1d>;
      };
  };
  ```
- 匹配规则：  
  `client->dev.of_node->compatible` 与 `of_match_table[i].compatible` 字符串 **前缀匹配**（支持通配符 `nxp,mma8*`）。

----------------------------------------------------------------
##### 六、ACPI 匹配（x86/服务器常见）  
```c
static const struct acpi_device_id mma8653_acpi_match[] = {
    { "NXPMMA8", 0 },   /* _HID 字符串 */
    { }
};
MODULE_DEVICE_TABLE(acpi, mma8653_acpi_match);

static struct i2c_driver mma8653_driver = {
    .driver = {
        .acpi_match_table = mma8653_acpi_match,
    },
};
```
- BIOS DSDT：  
  ```
  Device (I2C1)
  {
      Name (_HID, "NXPMMA8")  // _HID 必须全大写
  }
  ```
- 匹配规则：  
  `client->dev.acpi_companion->pnp.hardware_id` 与 `acpi_match_table[i].id` 字符串 **大小写不敏感** 比较。

----------------------------------------------------------------
##### 七、优先级与共存规则（内核源码）  
```c
static int i2c_device_match(struct device *dev, struct device_driver *drv)
{
    struct i2c_client *client = i2c_verify_client(dev);
    struct i2c_driver *driver = to_i2c_driver(drv);

    /* 1. Device Tree */
    if (of_driver_match_device(dev, drv))
        return 1;

    /* 2. ACPI */
    if (acpi_driver_match_device(dev, drv))
        return 1;

    /* 3. id_table */
    if (i2c_match_id(driver->id_table, client))
        return 1;

    return 0;
}
```
- 一旦 **Device Tree** 匹配成功，后两条不再检查。  
- 因此 **同一份驱动** 可同时维护三张表，平台无关。

----------------------------------------------------------------
##### 八、调试技巧速查  
| 需求                  | 方法                                                         |
| --------------------- | ------------------------------------------------------------ |
| 查看已匹配表          | `cat /sys/bus/i2c/devices/*/modalias`                        |
| 查看设备树 compatible | `cat /proc/device-tree/*/compatible`                         |
| 查看 ACPI _HID        | `cat /sys/bus/acpi/devices/*/hid`                            |
| 动态打印匹配过程      | `echo 'file i2c-core-base.c +p' > /sys/kernel/debug/dynamic_debug/control` |

----------------------------------------------------------------
一句话总结  
- **id_table**：老平台“名字+地址”硬编码；  
- **of_match_table**：设备树 `compatible` 字符串匹配，ARM/ARM64 首选；  
- **ACPI**：x86/服务器通过 `_HID/_CID` 匹配，BIOS 与驱动双向解耦。  

三条线可同时存在，**优先级 Device Tree > ACPI > id_table**，确保驱动 **一次编写，跨平台自动匹配**。

### 设备驱动源码追溯

#### I2C设备驱动注册

主调用链（以注册驱动后自动匹配为例）

- i2c_add_driver(driver)
  - 宏展开到 i2c_register_driver(THIS_MODULE, driver)
    - 设置 driver->driver.bus = &i2c_bus_type 等
    - 调用 driver_register(&driver->driver)
      - bus_add_driver(drv)
        - 若 bus->p->drivers_autoprobe 为真（默认是），执行：
          - driver_attach(drv)
            - bus_for_each_dev(bus, NULL, drv, __driver_attach)
              - __driver_attach(dev, drv)
                - 若匹配成功 driver_match_device(drv, dev)
                  - driver_probe_device(drv, dev)
                    - really_probe(dev, drv)
                      - 关键分发：
                        - 若 dev->bus->probe 存在：调用 dev->bus->probe(dev)
                        - 否则若 drv->probe 存在：调用 drv->probe(dev)



##### **i2c_add_driver**

```c
/* use a define to avoid include chaining to get THIS_MODULE */

#define i2c_add_driver(driver) \
	i2c_register_driver(THIS_MODULE, driver)

int i2c_register_driver(struct module *owner, struct i2c_driver *driver)
{
	int res;

	/* Can't register until after driver model init */
	if (unlikely(WARN_ON(!i2c_bus_type.p)))
		return -EAGAIN;

	/* add the driver to the list of i2c drivers in the driver core */
	driver->driver.owner = owner;
	driver->driver.bus = &i2c_bus_type;

	/* When registration returns, the driver core
	 * will have called probe() for all matching-but-unbound devices.
	 */
	res = driver_register(&driver->driver);
	if (res)
		return res;

	/* Drivers should switch to dev_pm_ops instead. */
	if (driver->suspend)
		pr_warn("i2c-core: driver [%s] using legacy suspend method\n",
			driver->driver.name);
	if (driver->resume)
		pr_warn("i2c-core: driver [%s] using legacy resume method\n",
			driver->driver.name);

	pr_debug("i2c-core: driver [%s] registered\n", driver->driver.name);

	INIT_LIST_HEAD(&driver->clients);
	/* Walk the adapters that are already present */
	i2c_for_each_dev(driver, __process_new_driver);

	return 0;
}
EXPORT_SYMBOL(i2c_register_driver);
```

###### i2c_add_driver

先采用宏定义的方式将i2c_add_driver(driver)链接成了i2c_register_driver(THIS_MODULE, driver)，且此处采用宏定义方式而不是内联，这样就不需要在 I2C 头文件里去包含定义 THIS_MODULE 的头文件（如 linux/module.h），从而避免“为拿到 THIS_MODULE 而层层包含头文件”的问题。

###### i2c_register_driver

在i2c_register_driver中，先检查I2C子系统是否初始化，即检查全局变量 `i2c_bus_type` 是否已初始化。

其中：

- `unlikely()` 是一个性能优化宏，提示编译器该条件很少会为真。
- `WARN_ON()` 是一个调试宏，如果条件为真，会在内核日志中打印警告信息。

然后设置驱动的基本信息，即设置驱动的所有者模块（`owner`）和所属总线（`i2c_bus_type`）。

随后调用内核的 `driver_register` 函数，将驱动注册到驱动核心（Driver Core），这里注册成功后，驱动核心会自动调用驱动的 `probe` 函数，为它绑定所有匹配的 I2C 设备。

随后便检查是否使用了旧的电源管理方法（若有则输出警告信息）和输出调试信息。

随后初始化驱动的 `clients` 链表，用于存储与该驱动绑定的 I2C 设备（clients）。

最后遍历适配器做遗留探测（detect）。

##### driger_register

```c
/**
 * driver_register - register driver with bus
 * @drv: driver to register
 *
 * We pass off most of the work to the bus_add_driver() call,
 * since most of the things we have to do deal with the bus
 * structures.
 */
int driver_register(struct device_driver *drv)
{
	int ret;
	struct device_driver *other;

	BUG_ON(!drv->bus->p);

	if ((drv->bus->probe && drv->probe) ||
	    (drv->bus->remove && drv->remove) ||
	    (drv->bus->shutdown && drv->shutdown))
		printk(KERN_WARNING "Driver '%s' needs updating - please use "
			"bus_type methods\n", drv->name);

	other = driver_find(drv->name, drv->bus);
	if (other) {
		printk(KERN_ERR "Error: Driver '%s' is already registered, "
			"aborting...\n", drv->name);
		return -EBUSY;
	}

	ret = bus_add_driver(drv);
	if (ret)
		return ret;
	ret = driver_add_groups(drv, drv->groups);
	if (ret)
		bus_remove_driver(drv);
	return ret;
}
EXPORT_SYMBOL_GPL(driver_register);
```

###### 函数定位与职责

- 位置：driver core（常见于 drivers/base/driver.c）
- 调用路径：子系统包装调用 → driver_register(&drv->driver) → bus_add_driver(drv)
  - 例：platform_driver_register → driver_register
- 职责：完成驱动在“总线级”的注册。核心初始化工作交给 bus_add_driver，随后补充创建该驱动声明的默认属性组。

driver_register 是驱动注册的入口薄封装：先做健全性与重名检查，再将工作交给 bus_add_driver，最后补充创建驱动声明的默认属性组；若属性组阶段失败则整体验证性回滚，保持设备模型一致、可预期。

##### 所以probe如何被调用？

really_probe 的核心逻辑：

- 如果总线（bus_type）提供了 .probe，就优先调用 bus 的 .probe
- 否则才直接调用通用的 device_driver.probe

这使得“总线可以插入一层适配/桥接”，把通用 device 指针和通用回调，转成该总线特定的 probe 形参与语义。

1. I2C 的专属桥接：bus->probe = i2c_device_probe

I2C 总线的 bus_type 通常定义在 drivers/i2c/i2c-core-base.c，形如：

```c
.match = i2c_device_match
.probe = i2c_device_probe
.remove = i2c_device_remove
```

i2c_device_probe 的典型步骤（简化示意）：
struct i2c_client *client = to_i2c_client(dev);
struct i2c_driver *drv = to_i2c_driver(dev->driver); // 通过 container_of 从通用 device_driver 找回外层 i2c_driver
如果 drv->probe_new 存在，调用 drv->probe_new(client)
否则根据 id_table 匹配得到 const struct i2c_device_id *id，再调用 drv->probe(client, id)
**关键点：**
i2c 驱动的“真正 probe 回调”存放在 struct i2c_driver 里（probe 或 probe_new），而不是放在通用的 device_driver.probe 里。
因此需要 bus 的 .probe（即 i2c_device_probe）做**“桥接/适配”**，把通用的 struct device_driver/struct device 转回 I2C 语义对象，并转调 i2c_driver 的 probe。
这也解释了“probe 是如何传到 really_probe 的”：实际上，probe 指针并没有直接“传进 really_probe”，而是 really_probe 里通过“先调用 bus->probe（i2c_device_probe）”，再由 bus->probe 通过 container_of 找回 i2c_driver，从而调用到 i2c_driver 的 .probe/.probe_new。

2. 两条常见触发路径

驱动注册时自动匹配（上面详述）
条件：bus->p->drivers_autoprobe = true（I2C 默认如此）
栈回溯示意：
i2c_add_driver → driver_register → bus_add_driver → driver_attach → driver_attach → driver_probe_device → really_probe → i2c_device_probe → i2c_driver->probe(_new)
设备后到（热插/晚注册）匹配
当新 I2C 设备被注册（例如 i2c_new_client_device/of/ACPI 枚举到）时，device_add 会触发与已注册驱动匹配，之后路径会在 dd.c 中走到 device_attach/device_attach_driver/driver_probe_device/really_probe，最终仍是 really_probe → i2c_device_probe → i2c_driver->probe(_new)。

3. 与 device_driver.probe 的关系（对比说明）

模型里有两种模式：
Bus 层提供 bus->probe（I2C、USB 等常见）：really_probe 先走 bus->probe，bus 层做语义适配，调用“总线自有的 driver 结构体里的 probe”（如 i2c_driver->probe）
Bus 层不提供 bus->probe：驱动把通用的 drv->probe 设为一个总线适配的包装器（wrapper），really_probe 会直接调 drv->probe，wrapper 再做 container_of 和类型转换
I2C 采用的是第 1 种，因此 i2c_driver 一般不设置 driver.driver.probe，避免旧内核里出现“同时设置 bus->probe 与 drv->probe 的警告”。

4. 小结

i2c_add_driver 通过 driver_register → bus_add_driver，把驱动挂到 I2C 总线上；若启用 autoprobe，就会立即遍历现有设备并尝试绑定。
真正调用你驱动里 probe 的路径会经过：
driver_probe_device → really_probe → dev->bus->probe（i2c_device_probe）→ i2c_driver->probe 或 probe_new
“probe 如何传到 really_probe”：不是把 i2c_driver->probe 指针直接传入，而是 really_probe 根据“bus 是否定义 .probe”来决定调用 i2c_device_probe；该函数利用 dev->driver 指向的通用 device_driver，container_of 回到外层 i2c_driver，再转调 i2c_driver 的 probe。这就是“总线层桥接”的关键。

#### 那么i2c_client是在哪里出生的？

以目前研究的6818中的板级文件方式为例

在arch\arm\plat-s5p6818\x6818\devices.c中已经写好了mma8653的i2c_board_info，设备初始化加载时就已经通过该板级文件调用i2c_register_board_info(2, &mma865x_i2c_bdi, 1);把其注册在编号为2的i2c适配器下当 i2c-2 控制器调用 i2c_register_adapter() 时，内核自动执行 i2c_scan_static_board_info()，遍历链表并匹配 busnum == 2 的条目，创建 i2c_client。

#### SMBus 调用链路

i2c_smbus_read_byte_data(client, reg)  // 用户调用
  └── i2c_smbus_xfer()  // drivers/i2c/i2c-core-smbus.c
      ├── 检查 adap->algo->smbus_xfer
      └── 为 NULL → 调用 i2c_smbus_xfer_emulated()  // 软件模拟
          └── 构造 I2C 消息数组（写地址+读数据）
              └── i2c_transfer(adap, msgs, num)  // 回退到 master_xfer
                  └── adap->algo->master_xfer(adap, msgs, num)  // 调用 i2c-gpio 的位 bang 实现
                      └── i2c_gpio_xfer()  // 最终执行 GPIO 模拟时序

### 适配器驱动源码追溯

源码目录：drivers\i2c\busses

#### i2c_gpio_private_data结构体

```c
struct i2c_gpio_private_data {
	struct i2c_adapter adap;
	struct i2c_algo_bit_data bit_data;
	struct i2c_gpio_platform_data pdata;
};
```



##### 总体说明

- 这是驱动的“私有包”——把与一个 i2c-gpio 实例相关的三个关键对象放在一个连续的结构体里。probe 中用 devm_kzalloc 分配这块内存，并通过 platform_set_drvdata 挂到 platform_device 上，remove 时通过 platform_get_drvdata 取回，便于管理生命周期和错误回滚。三个成员都是嵌入式对象（不是指针），因此它们的地址在内存中稳定，可以直接取地址传给内核其它子系统（比如 i2c 核心或回调）。

##### struct i2c_adapter adap

- 在本驱动中的作用
  - 表示该驱动向 I2C 子系统注册的适配器（即一条 I2C 总线控制器的抽象）。
  - 通过填充 adap 的若干字段并注册，内核及上层 I2C 驱动就能通过该适配器进行 I2C 传输并在 sysfs 中看到该总线。
- 在代码中的具体用途（引用点）
  - 在 probe 中填充字段：
    - adap->owner = THIS_MODULE;
    - snprintf(adap->name, ... , "i2c-gpio%d", pdev->id);
    - adap->algo_data = bit_data; // 把位算法数据指给适配器
    - adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
    - adap->dev.parent = &pdev->dev;
    - adap->dev.of_node = pdev->dev.of_node;
    - adap->nr = (pdev->id != -1) ? pdev->id : 0;
  - 调用 i2c_bit_add_numbered_bus(adap) 注册适配器：
    - i2c_bit_add_numbered_bus 会把 adap->algo 设为 i2c-algo-bit 提供的算法，并把适配器加入 I2C 框架（使其可用）。
  - remove 时通过 i2c_del_adapter(&priv->adap) 注销适配器。
- 为什么放在私有结构体中
  - 适配器对象需要和对应的 GPIO/平台数据关联（在 remove 时需要），将其与其它数据放在同一私有块便于管理与 cleanup。
  - i2c 核心会持有对 adap 的引用（master_xfer 路径），所以应保持内存直到 unregister。

##### struct i2c_algo_bit_data bit_data

- 在本驱动中的作用
  - 这是 i2c-algo-bit（通用位操作 I2C 算法）期望的参数/回调容器。它包含 setsda/setscl/getsda/getscl 回调指针、时序 udelay、timeout、以及一个 data 指针用于回调访问平台数据。
  - 驱动通过填充该结构把 GPIO 操作细节交给 i2c-algo-bit，由后者实现 I2C 协议（START/STOP、读写位、ACK、时钟拉伸等待等）。
- 在代码中的具体用途（引用点）
  - 在 probe 中被填充：
    - bit_data->setsda = i2c_gpio_setsda_val 或 i2c_gpio_setsda_dir（根据 pdata 配置选择）
    - bit_data->setscl = i2c_gpio_setscl_val 或 i2c_gpio_setscl_dir
    - 如果允许时钟拉伸则 bit_data->getscl = i2c_gpio_getscl
    - bit_data->getsda = i2c_gpio_getsda
    - bit_data->udelay = pdata->udelay 或 默认值 (5 或 50)
    - bit_data->timeout = pdata->timeout 或 默认 HZ/10
    - bit_data->data = pdata; // 把平台数据地址供回调使用
  - 把 bit_data 的地址放到 adap->algo_data（adap->algo_data = bit_data），i2c-algo-bit 在运行时会通过 adapter->algo_data 拿到这些回调和参数。
- 为什么放在私有结构体中
  - bit_data 引用 pdata（通过 bit_data->data = pdata），且要与 adap 一起长期存在直到适配器注销；放在同一私有块可保证一致的生命周期和内存连贯性。
  - 回调指针指向静态函数（定义在同一源文件），而 data 指向私有 pdata，因此 embedding 避免了额外分配与指针管理。

##### struct i2c_gpio_platform_data pdata

- 在本驱动中的作用
  - 存储平台/设备树提供的配置信息：SDA/SCL 的 GPIO 编号、是否为开漏、是否 SCL 仅输出、udelay、timeout 等。这些是驱动配置与回调运行时需要的原始参数。
- 在代码中的具体用途（引用点）
  - 在 probe 的开始由 of_i2c_gpio_probe(...)（设备树）或 memcpy(platform_data) 填充 pdata：
    - pdata->sda_pin、pdata->scl_pin
    - pdata->sda_is_open_drain、pdata->scl_is_open_drain、pdata->scl_is_output_only
    - pdata->udelay、pdata->timeout（转换后的 jiffies）
  - probe 使用 pdata 的内容来：
    - 调用 gpio_request(pdata->sda_pin/scl_pin) 申请 GPIO。
    - 根据 open-drain/输出标志设置 gpio_direction_input/output 并选择对应的 setsda/setscl 回调。
    - 决定是否设置 getscl（基于 scl_is_output_only）。
    - 决定 bit_data->udelay 与 bit_data->timeout 的值（若 pdata 指定则使用）。
  - bit_data->data = pdata：把 pdata 的地址传给位算法回调，回调通过 (struct i2c_gpio_platform_data *)data 读取 pin 编号并调用 gpio_set_value/gpio_direction/gpio_get_value。
- 为什么放在私有结构体中
  - pdata 是与该适配器紧密绑定的配置信息，回调在运行时需要访问它。把它放在私有结构体保证回调取到的数据在适配器生命周期内有效。
  - 也避免了对 platform_data 进行额外复制/分配，保证所有相关数据放在一处。

##### 相互关系与指针流向（关键点）

- 内存布局：priv（devm 分配）包含 adap、bit_data、pdata，地址稳定。
- adapter -> algo_data = &priv->bit_data：i2c-algo-bit 通过 adapter->algo_data 访问回调/参数。
- bit_data->data = &priv->pdata：回调收到 data 参数后能读取具体的 GPIO 编号和标志以进行 gpio_set/get/direction。
- 回调函数（如 i2c_gpio_setsda_val/dir 等）原型是 (void *data, int state)；它们会把 void *data 强转为 struct i2c_gpio_platform_data * 并使用 pdata->sda_pin/scl_pin。
- platform_set_drvdata(pdev, priv)：把 priv 关联到 platform_device，remove 时通过 platform_get_drvdata 取回，进而调用 i2c_del_adapter(&priv->adap) 和 gpio_free(pdata->*); 所以 priv 必须在 probe 成功后存在直到 remove。

##### 生命周期与资源管理

- 内存（priv）分配：devm_kzalloc，随 device 释放自动 free（不需显式 free）。
- GPIO 资源申请：gpio_request（不是 devm），因此在错误或 remove 时需要显式 gpio_free。probe 的错误路径正确释放已申请的 GPIO。
- 适配器注册：i2c_bit_add_numbered_bus 成功后，适配器被内核持有；remove 时通过 i2c_del_adapter 注销。
- 因为 adap、bit_data、pdata 都嵌入在同一私有块中，保证在适配器活跃期间这些数据地址始终有效，避免悬空引用。

#### 驱动代码中的两组回调区别

- val（value）方式通过写输出电平（gpio_set_value）来控制线的高低。
- dir（direction）方式通过改变引脚方向（gpio_direction_input / gpio_direction_output(..., 0)）来控制线：输入表示释放线（高阻），输出且写 0 表示拉低。

下面详细解释、对比和影响。

##### 原理对比

- val（写值）：
  - 假设 GPIO 配置为开漏（open-drain）或被配置为“可以安全写 1 表示释放”的模式。
  - gpio_set_value(pin, 0) -> 驱动输出低（拉低线）
  - gpio_set_value(pin, 1) -> 释放输出驱动（对于开漏此时是高阻，外部上拉把线拉高）
- dir（切换方向）：
  - 通过方向切换来模拟开漏：
    - gpio_direction_output(pin, 0) -> 主动拉低
    - gpio_direction_input(pin) -> 变为输入（高阻），外部上拉把线拉高
  - 适用于那些不支持硬件开漏但可以通过输入（高阻）/输出0 模拟开漏的 GPIO。

##### 代码中何时用哪种方式

- 如果平台/设备树表明引脚是真正的 open-drain（sda_is_open_drain / scl_is_open_drain），驱动会把引脚配置为输出并用 sets*_val（gpio_set_value）。
- 如果不是 open-drain，驱动用 sets*_dir（通过改变方向来实现释放/拉低）。
- 对 SCL，还有一个特殊标志 scl_is_output_only：若为真，驱动把 SCL 设为输出并用 setscl_val（不能读 SCL，也就不支持时钟拉伸检测）。

##### 电气与兼容性差别

- val 合法条件：
  - GPIO 硬件/控制器实际工作在开漏或能够在写 1 时断开驱动（高阻）。很多 SoC 的 GPIO 控制器支持“open-drain”属性，或者在设备树中能声明为 open-drain。
  - 如果 GPIO 是推挽（push-pull）且你用 gpio_set_value(pin, 1)，会主动驱动高电平，这会和其他设备（或从设备想拉低线）冲突，破坏 I2C 总线。
- dir 的优点/缺点：
  - 优点：能在不支持开漏输出的 GPIO 上工作（用方向切换实现）。更“通用”。
  - 缺点：依赖外部上拉电阻；方向切换开销（有时比单纯写值慢）；在某些 GPIO 控制器上，切换方向/读取瞬间状态可能有微妙时序差异或寄存器语义问题。
- 时钟拉伸：
  - 若 SCL 是 output-only（只输出），驱动不会提供 getscl（读 SCL），i2c-algo-bit 不会等待从机释放 SCL（即不支持钟拉伸）。这通常与使用 setscl_val（输出写值）相关联。
  - 若用 dir 且能读取 SCL（getscl 可用），算法能检测从机拉低 SCL（支持时钟拉伸）。

#### i2c_gpio_probe

```c
static int __devinit i2c_gpio_probe(struct platform_device *pdev)
{
	struct i2c_gpio_private_data *priv;
	struct i2c_gpio_platform_data *pdata;
	struct i2c_algo_bit_data *bit_data;
	struct i2c_adapter *adap;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	adap = &priv->adap;
	bit_data = &priv->bit_data;
	pdata = &priv->pdata;

	if (pdev->dev.of_node) {
		ret = of_i2c_gpio_probe(pdev->dev.of_node, pdata);
		if (ret)
			return ret;
	} else {
		if (!pdev->dev.platform_data) 
			return -ENXIO;
		memcpy(pdata, pdev->dev.platform_data, sizeof(*pdata));
	}

	ret = gpio_request(pdata->sda_pin, "sda");
	if (ret)
		goto err_request_sda;
	ret = gpio_request(pdata->scl_pin, "scl");
	if (ret)
		goto err_request_scl;

	if (pdata->sda_is_open_drain) {
		gpio_direction_output(pdata->sda_pin, 1);
		bit_data->setsda = i2c_gpio_setsda_val;
	} else {
		gpio_direction_input(pdata->sda_pin);
		bit_data->setsda = i2c_gpio_setsda_dir;
	}

	if (pdata->scl_is_open_drain || pdata->scl_is_output_only) {
		gpio_direction_output(pdata->scl_pin, 1);
		bit_data->setscl = i2c_gpio_setscl_val;
	} else {
		gpio_direction_input(pdata->scl_pin);
		bit_data->setscl = i2c_gpio_setscl_dir;
	}

	if (!pdata->scl_is_output_only)
		bit_data->getscl = i2c_gpio_getscl;
	bit_data->getsda = i2c_gpio_getsda;

	if (pdata->udelay)
		bit_data->udelay = pdata->udelay;
	else if (pdata->scl_is_output_only)
		bit_data->udelay = 50;			/* 10 kHz */
	else
		bit_data->udelay = 5;			/* 100 kHz */

	if (pdata->timeout)
		bit_data->timeout = pdata->timeout;
	else
		bit_data->timeout = HZ / 10;		/* 100 ms */

	bit_data->data = pdata;

	adap->owner = THIS_MODULE;
	snprintf(adap->name, sizeof(adap->name), "i2c-gpio%d", pdev->id);
	adap->algo_data = bit_data;
	adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;

	/*
	 * If "dev->id" is negative we consider it as zero.
	 * The reason to do so is to avoid sysfs names that only make
	 * sense when there are multiple adapters.
	 */
	adap->nr = (pdev->id != -1) ? pdev->id : 0;
	ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
		goto err_add_bus;

	of_i2c_register_devices(adap);

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "using pins %u (SDA) and %u (SCL%s)\n",
		 pdata->sda_pin, pdata->scl_pin,
		 pdata->scl_is_output_only
		 ? ", no clock stretching" : "");

	return 0;

err_add_bus:
	gpio_free(pdata->scl_pin);
err_request_scl:
	gpio_free(pdata->sda_pin);
err_request_sda:
	return ret;
}
```

##### 函数目的

- 把一个 platform 设备（可能由设备树/ACPI/板级代码创建）绑定成一个“基于 GPIO 的 I2C 主机适配器”，并注册到 I2C 核心。
- 关键工作：解析配置（GPIO 与电气/时序属性）、申请并初始化 GPIO、把 GPIO 操作回调装配进 i2c-algo-bit 的位操作算法、注册适配器并枚举子设备、建立驱动私有数据，处理错误路径释放资源。

##### 逐块解析

###### 1. 分配并初始化私有数据

- 代码:
  - priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
  - adap = &priv->adap; bit_data = &priv->bit_data; pdata = &priv->pdata;
- 含义与原因:
  - 使用 devm_kzalloc 分配“设备管理的”零填充内存：随 device 生命周期自动释放，减少手动清理负担（但注意 GPIO 是非 devm 申请的，后面会手动 free）。
  - 将三个紧密关联的对象打包：
    - adap: I2C 适配器对象，注册到 I2C 核心。
    - bit_data: 给 i2c-algo-bit 的回调与参数。
    - pdata: 平台参数（GPIO 编号、电气属性、时序参数）。
- 失败处理:
  - 分配失败返回 -ENOMEM。

###### 2.获取平台配置（设备树 or 传统 platform_data）

- 代码:
  - if (pdev->dev.of_node) ret = of_i2c_gpio_probe(...);
  - else 从 pdev->dev.platform_data memcpy 到 pdata；若没有则返回 -ENXIO。
- 含义与原因:
  - 设备树路径：从 OF 节点解析 SDA/SCL GPIO、开漏/仅输出属性、延时/超时等，统一填入 pdata。
  - 非设备树路径：依赖平台代码提供 struct i2c_gpio_platform_data。（arch/arm/mach-s5p6818/devices.c）
- 错误条件:
  - 设备树缺关键属性或解析失败，返回解析的错误码。
  - 非 OF 且没有 platform_data，返回 -ENXIO（设备不存在/无法驱动）。

###### 3. 申请 GPIO 资源

- 代码:
  - gpio_request(pdata->sda_pin, "sda")
  - gpio_request(pdata->scl_pin, "scl")
- 含义与原因:
  - 向 GPIO 子系统声明占用 SDA/SCL 引脚，便于调试和避免冲突。"sda"/"scl" 作为标签用于调试可见性。
- 错误路径:
  - 若申请 SDA 失败，跳转 err_request_sda 直接返回。
  - 若申请 SCL 失败，跳转 err_request_scl，释放已申请的 SDA 后返回。

###### 4. 按电气属性配置 GPIO 方向/初值并选择“如何拉高/拉低”的回调

- SDA 配置:
  - 若 sda_is_open_drain:
    - gpio_direction_output(sda, 1); bit_data->setsda = i2c_gpio_setsda_val;
    - 逻辑：开漏输出模式，输出“1”表示释放（高阻，由外部上拉拉高），输出“0”表示拉低。
  - 否则:
    - gpio_direction_input(sda); bit_data->setsda = i2c_gpio_setsda_dir;
    - 逻辑：用“切换方向”模拟开漏：输入=高阻（释放），输出0=拉低。
- SCL 配置:
  - 若 scl_is_open_drain 或 scl_is_output_only:
    - gpio_direction_output(scl, 1); bit_data->setscl = i2c_gpio_setscl_val;
    - 说明：开漏或“仅输出”都通过写值拉高/拉低。仅输出意味着后续不支持时钟拉伸。
  - 否则:
    - gpio_direction_input(scl); bit_data->setscl = i2c_gpio_setscl_dir;
    - 说明：非开漏时同样依赖“切换方向”实现释放/拉低。
- 为什么分两套策略:
  - 真·开漏硬件：可以通过写值直接释放/拉低，简单可靠。
  - 非开漏硬件：通过方向切换达到“高阻/拉低”的等效效果，满足 I2C 线与逻辑。
- gets 回调:
  - 若不是 scl-output-only：bit_data->getscl = i2c_gpio_getscl（用于检测时钟拉伸）。
  - 始终设置 getsda = i2c_gpio_getsda（用于读位/ACK 检测）。

###### 5. 设置位时序参数 udelay 与超时 timeout

- 代码:
  - udelay:
    - 若 pdata->udelay 指定则使用；
    - 否则：scl-output-only => 50us（约 10 kHz）；否则 5us（约 100 kHz）。
  - timeout:
    - 若 pdata->timeout 指定则使用；
    - 否则默认 HZ/10（约 100ms）。
- 含义与原因:
  - udelay 控制位翻转之间的延迟，决定 I2C 近似速率；仅输出 SCL 时选择更慢的默认值以提升兼容性。
  - timeout 用于等待 SCL 变高（时钟拉伸）等超时控制；单位是 jiffies（设备树解析中若给了毫秒会先转 jiffies）。
- bit_data->data = pdata:
  - 把平台参数指针传给位回调，回调通过 data 拿到 gpio 编号等。

###### 6. 填充适配器并注册为 I2C 总线

- 代码:
  - adap->owner = THIS_MODULE
  - snprintf(adap->name, "i2c-gpio%d", pdev->id)
  - adap->algo_data = bit_data
  - adap->class = I2C_CLASS_HWMON | I2C_CLASS_SPD
  - adap->dev.parent = &pdev->dev; adap->dev.of_node = pdev->dev.of_node
  - adap->nr = (pdev->id != -1) ? pdev->id : 0
  - ret = i2c_bit_add_numbered_bus(adap)
- 含义与原因:
  - owner 指向当前模块，防止卸载中使用。
  - name 为调试友好的适配器名称。
  - algo_data 告诉 i2c-algo-bit 我们的 GPIO 回调和参数在哪。
  - class 标注此总线常见的设备类别（硬件监控 HWMON 和 SPD 内存条），便于某些自动探测或驱动策略。
  - parent/of_node 关联设备树节点，使后续 of_i2c_register_devices 能找到并实例化子节点。
  - nr 指定“编号总线”（如 i2c-<nr>）；若 pdev->id 为 -1 则强制用 0，避免只有一个适配器时生成奇怪的 sysfs 名称。
  - i2c_bit_add_numbered_bus 的关键作用：
    - 内部会把 adap->algo 设置为 i2c-algo-bit 提供的通用算法（i2c_bit_algo），并完成适配器注册。
    - 之后 I2C 核心的 master_xfer 将由该算法实现，算法再回调我们提供的 setsda/setscl/getsda/getscl 来“抖”GPIO。

###### 7. 枚举并注册设备树子设备

- 代码:
  - of_i2c_register_devices(adap)
- 含义与原因:
  - 扫描适配器的 of_node 下的子节点（即挂在此 I2C 总线上的从设备），为每个子节点创建 i2c_client 并绑定相应驱动。

###### 8. 建立驱动私有数据并打印信息

- 代码:
  - platform_set_drvdata(pdev, priv)
  - dev_info(..., "using pins %u (SDA) and %u (SCL%s)\n", ..., pdata->scl_is_output_only ? ", no clock stretching" : "")
- 含义与原因:
  - 将 priv 存入平台设备，供 remove 时取回。
  - 打印所用引脚及是否禁用“时钟拉伸”（scl-output-only），便于排错。

###### 9. 成功返回与错误回滚路径

- 正常返回 0。
- 错误回滚:
  - err_add_bus: gpio_free(scl) -> fallthrough 到 err_request_scl
  - err_request_scl: gpio_free(sda) -> fallthrough 到 err_request_sda
  - err_request_sda: return ret
- 说明:
  - 典型“台阶式”回滚：每失败一步释放之前成功的资源，避免泄漏。
  - 注意 GPIO 不是 devm 申请，必须手动释放；内存是 devm 分配，会自动回收。

##### 与运行时的关系

- 注册成功后，任何对该适配器的 I2C 传输请求都会走 i2c-algo-bit 的 master_xfer。
- master_xfer 利用本函数设置的 bit_data（回调与参数），调 setsda/setscl/getsda/getscl 产生 I2C 起始/停止、数据/ACK、以及（若可）时钟拉伸等待，udelay 决定节拍、timeout 控制等待上限。

##### 重要函数

###### i2c_bit_add_numbered_bus( )

```c
ret = i2c_bit_add_numbered_bus(adap);
	if (ret)
		goto err_add_bus;
```

- 含义
  - 调用 i2c-algo-bit 提供的 helper 函数，将你的 adapter 注册为 bit-banged（位操作）I2C 总线并请求分配/占用 adap->nr 号。
- i2c_bit_add_numbered_bus 做了什么（内部要点）
  - 将 adap->algo 指向 i2c-algo-bit 中实现的通用算法结构（如 i2c_bit_algo）；
  - 使用 adap->algo_data（你已设置为指向 bit_data）让算法知道如何操作 SDA/SCL；
  - 调用 i2c_add_numbered_adapter(adap)（或等价流程）把适配器加入 I2C 子系统，并尝试使用 adap->nr 指定的总线号；
  - 做必要的初始化工作以便 master_xfer 能调度到 bit 算法。
- 返回值/错误处理
  - 成功返回 0；失败返回负 errno（例如总线号冲突、资源不足等）。
  - 若失败，代码跳转到 err_add_bus 标签执行资源回滚（释放已 request 的 GPIO），然后返回错误。
- 语义影响
  - 在这行成功后，适配器被内核 I2C 子系统登记，外部（其它驱动/内核子系统）可以通过该适配器进行 I2C 传输；同时适配器可能在 sysfs 中出现（i2c-<nr>）并可被用户空间（i2c-dev 等）访问。

###### of_i2c_register_devices()

- 含义
  - 如果适配器对应的 device 树节点下有子节点（描述挂在这条 I2C 总线上的从设备），该函数会解析这些子节点并为它们创建对应的 i2c_client（board info -> i2c_new_device），以便与相应的从设备驱动匹配并绑定。
- 为什么要在 i2c_bit_add_numbered_bus 之后调用
  - of_i2c_register_devices 需要一个已经注册到 I2C 子系统的 adapter（即内核已经知道这条总线，并能为新创建的 i2c_client 找到它对应的 adapter）。如果先注册子设备，而适配器尚未注册，设备绑定可能失败或被延迟。
- 作用细节
  - 解析 device tree 子节点（通常会从子节点读取 reg/address、compatible、驱动数据等），生成 struct i2c_board_info（或 of equivalent），然后调用 i2c_new_device（或 of_i2c_register_board_info 风格的接口）创建 i2c_client。
  - 如果没有任何子节点，函数几乎没有行为（无错误）。
- 后果
  - 使得用设备树描述的 I2C 从设备能自动被内核实例化并与驱动匹配（例如某个传感器的驱动会 probe）。

###### platform_set_drvdata(pdev, priv);

- 含义
  - 把本 probe 中分配的私有结构体 priv（包含 adap、bit_data、pdata）与 platform_device 关联起来，内核会把它保存在 pdev->dev.driver_data 中。
- 目的
  - 在 remove（卸载驱动或设备）或其他需要访问私有数据的地方，可以通过 platform_get_drvdata(pdev) 取回 priv，从而进行清理（例如 i2c_del_adapter / gpio_free）或状态查询。
- 生命周期/约定
  - 在 probe 成功返回后，驱动应保证在 remove 时通过 platform_get_drvdata 取得同一 priv 并释放资源。
  - 这是典型的驱动私有数据存储模式。

