#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/interrupt.h>
#include <mach/s5p6818_irq.h>
#include <mach/irqs.h>
#include <cfg_type.h> 

	
irqreturn_t key_handler(int irq,void *dev)
{
	printk(KERN_INFO"key put down, irq: %d \n",irq);


	return IRQ_HANDLED;
}



static int __init intr_init(void)
{
	int ret;

	printk(KERN_WARNING"bound intr init\n");

	ret = request_irq((IRQ_GPIO_B_START+30), key_handler, IRQF_DISABLED|IRQF_TRIGGER_FALLING, "key-intr", NULL);
	if(ret < 0)
	{
		printk(KERN_INFO"request irq failed\n");
		return -1;
	}
	return 0;
}

static void __exit intr_exit(void)
{
	printk(KERN_WARNING"bound intr exit\n");

	free_irq((IRQ_GPIO_B_START+30),NULL);
	
	return ;
}


module_init(intr_init);

module_exit(intr_exit);


MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code, intr part");
MODULE_VERSION("1.0");
