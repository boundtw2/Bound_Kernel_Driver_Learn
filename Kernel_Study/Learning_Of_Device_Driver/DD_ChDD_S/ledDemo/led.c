/*
 * LED1->GPIOB26	
 * LED2->GPIOC11
 * LED3->GPIOC7
 * LED4->GPIOC12
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>

#include <linux/ioport.h>
#include <asm/io.h>

#include <linux/fs.h>
#include <asm/uaccess.h>


#define LED1 0x1 
#define LED2 0x2 
#define LED3 0x3
#define LED4 0x4 


#define LED_ON 	0xe
#define LED_OFF	0xf



int kstatus;

struct resource *res = NULL;
unsigned long *gpioc_base = NULL;
unsigned long *gpiob_base = NULL;

void led_delay(volatile int count)
{
        while (count--);
}

void led_init(void)
{
	printk(KERN_INFO "led init...\n");
   	 
	//GPIOB11-12(LED2,4) -> OUTPUT, LOW
	*(gpioc_base + 1) |= (0x3<<11);
	*gpioc_base &= ~(0x3<<11);
	
	//GPIOC7(LED3) -> OUTPUT, LOW
	*(gpioc_base + 1) |= (0x1<<7);
	*gpioc_base &= ~(0x1<<7);

	//GPIOB26(LED1) -> OUTPUT, LOW
	*(gpiob_base + 1) |= (0x1<<26);
	*gpiob_base &= ~(0x1<<26);

}

int led_open(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "led open ...\n");
	led_init();
	
	return 0;
}

int led_close(struct inode *inode, struct file *file)
{
	printk(KERN_INFO "led close ...\n");
	return 0;
}

int led_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	printk(KERN_INFO "led ioctl\n");
	switch(arg)
	{
		case LED1:
			if(cmd == LED_ON)
				*gpiob_base &= ~(0x1<<26);
			else
				*gpiob_base |= (0x1<<26);
			break;
		case LED2:
			if(cmd == LED_ON)
				*gpioc_base &= ~(0x1<<11);
			else
				*gpioc_base |= (0x1<<11);
			break;
		case LED3:
			if(cmd == LED_ON)
				*gpioc_base &= ~(0x1<<7);
			else
				*gpioc_base |= (0x1<<7);
			break;
		case LED4:
			if(cmd == LED_ON)
				*gpioc_base &= ~(0x1<<12);
			else
				*gpioc_base |= (0x1<<12);
			break;
	}
}


ssize_t led_read(struct file *file, char __user *buf, size_t size, loff_t *offset)
{
	
	if(*gpioc_base &(0x1<<11))
	{

		kstatus = 1;
	}
	else 
	{
		kstatus = 0;
	}

	copy_to_user(buf, &kstatus, size);
}


struct file_operations led_fops = {
	.open 		= led_open,
	.release 	= led_close,
	.compat_ioctl 	= led_ioctl,
	.unlocked_ioctl = led_ioctl, 

	.read 		= led_read,
};

int mydrv_init(void)
{
	printk(KERN_INFO "hello,led.\n");

	int err;
	gpioc_base = (unsigned long *)ioremap(0xc001c000, 0x18);
	if(gpioc_base == NULL)
	{
		printk(KERN_INFO "ioremap gpiocbase addr fail\n");
		return -EBUSY;
	}
	gpiob_base = (unsigned long *)ioremap(0xc001b000, 0x18);

	err = register_chrdev(88, "myled", &led_fops);
	if(err)
	{
		printk(KERN_INFO "register chrdev fail\n");
		return -EBUSY;
	}

	return 0;
}

void mydrv_exit(void)
{
	printk(KERN_INFO "bye,kernel.\n");

	unregister_chrdev(88, "myled");
	iounmap(gpioc_base);
	iounmap(gpiob_base);
	
}

module_init(mydrv_init);
module_exit(mydrv_exit);



