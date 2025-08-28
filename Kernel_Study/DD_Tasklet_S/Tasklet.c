#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/interrupt.h>
#include <mach/s5p6818_irq.h>
#include <mach/irqs.h>
#include <cfg_type.h> 
#include <linux/delay.h>

struct tasklet_struct mytask;

irqreturn_t key_handler(int irq,void *dev)
{
	printk(KERN_INFO"key put down, irq: %d \n",irq);

	tasklet_schedule(&mytask);

	printk(KERN_INFO"tasklet goto ending\n");

	return IRQ_HANDLED;
}

void mytask_handler(unsigned long data)
{
	int cnt = 10;

	printk(KERN_INFO"tasklet data: %lu",data);
	
	while(cnt--)
	{
		printk(KERN_INFO"looping......\n");
		mdelay(500);
	}

}

static int __init tl_init(void)
{
	int ret;

	printk(KERN_WARNING"bound intr init\n");

	ret = request_irq((IRQ_GPIO_B_START+30), key_handler, IRQF_TRIGGER_FALLING, "key-intr", NULL);
	if(ret < 0)
	{
		printk(KERN_INFO"request irq failed\n");
		return -1;
	}

	tasklet_init(&mytask, mytask_handler, 88);

	while(1)
	{
		printk(KERN_INFO"main kthread running\n");
		mdelay(1000);
	}
	
	return 0;
}

static void __exit tl_exit(void)
{
	printk(KERN_WARNING"bound intr exit\n");

	free_irq((IRQ_GPIO_B_START+30),NULL);

	tasklet_kill(&mytask);
	
	return ;
}


module_init(tl_init);

module_exit(tl_exit);


MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code, tasklet part");
MODULE_VERSION("1.0");
