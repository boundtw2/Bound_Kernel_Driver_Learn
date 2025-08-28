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

​	一个I2C适配器需要i2c i2c_algorithm提供的通信函数来控制适配器产生特定的访问周期。缺少i2c_algorithm的i2c_adapter什么也做不了，因此i2c_adapter中包含所使用的i2c_algorithm 的指针。

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

> 关键点：
>
> - 注册时**立即遍历**所有已存在的 adapter，尝试 detect 并创建 client；
> - 如果设备树或 ACPI 已描述，则 detect 阶段直接匹配，不再做地址扫描。

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