#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/ioport.h>
#include <asm/io.h>


#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>


dev_t dev_id; //主设备号
struct cdev myled_cdev; //字符设备结构体
static struct class  *myled_class;

unsigned long *gpioc_base = NULL;

void led_init(void)
{
	printk(KERN_INFO "led init...\n");
   	 
	//GPIOB12 LED4 -> OUTPUT, HIGH
	*(gpioc_base + 1) |= (0x1<<12);
	*gpioc_base |= (0x0<<12);

	
}

void led_exit(void)
{
	printk(KERN_INFO "led exit...\n");
   	 
	//GPIOB12 LED4 ->  LOW, INPUT
	*gpioc_base &= ~(0x1<<12);
	*(gpioc_base + 1) &= ~(0x1<<12);
	
}


int myled_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "myled open\n");
	led_init();

	return 0;
}	

int myled_close(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "myled release\n");
	led_exit();
	
	return 0;
}

struct file_operations led_fops = {
	.open = myled_open,
	.release = myled_close,

};

static int __init mydrv_init(void)
{

	int err;

	printk(KERN_INFO " led init\n");

	//静态注册
	/*
	err = register_chrdev(88, "myled-dev", &led_fops);
	if (err < 0) {
		printk(KERN_ERR "Failed to register char device\n");
		return -1;
	}
	*/
	
	gpioc_base = (unsigned long *)ioremap(0xc001c000, 0x18);
	if(gpioc_base == NULL)
	{
		printk(KERN_INFO "ioremap gpiocbase addr fail\n");
		return -EBUSY;
	}
	
	err = alloc_chrdev_region(&dev_id, 56, 2, "myled");
	if (err < 0) {
		printk(KERN_ERR "Failed to allocate char device region\n");
		return -1;
	}
	printk(KERN_INFO "dev_id_all: %u, major:%d, minor:%d\n",dev_id, MAJOR(dev_id), MINOR(dev_id));

	cdev_init(&myled_cdev, &led_fops);

	err = cdev_add(&myled_cdev, dev_id, 2);
	if (err < 0) {
		printk(KERN_ERR "Failed to add char device\n");
		 goto err_cdev;
	}

	/* 3. 创建 class（/sys/class/myled） */
    myled_class = class_create(THIS_MODULE, "myled");
    if (IS_ERR(myled_class)) {
        err = PTR_ERR(myled_class);
        goto err_class;
    }

    /* 4. 在 class 下创建设备（/dev/myled） */
    device_create(myled_class, NULL, MKDEV(MAJOR(dev_id), 0), NULL, "myled");
	
	return 0;

err_class:
	cdev_del(&myled_cdev);
err_cdev:
	unregister_chrdev_region(dev_id, 2);
	return err;
}

static void __exit mydrv_exit(void)
{
	printk(KERN_INFO "led exit\n");

	// unregister_chrdev(88, "myled-dev");
	iounmap(gpioc_base);
	device_destroy(myled_class,dev_id);
	class_destroy(myled_class);
	
	unregister_chrdev_region(dev_id, 2);
	cdev_del(&myled_cdev);
	
	return;
}

module_init(mydrv_init);
module_exit(mydrv_exit);


MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code,  char driver part");
MODULE_VERSION("1.0");
