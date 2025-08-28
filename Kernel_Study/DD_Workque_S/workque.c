#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <mach/s5p6818_irq.h>
#include <cfg_type.h> 
#include <linux/workqueue.h>
#include <linux/delay.h>


static struct work_data wkdata;

struct work_data
{
	struct work_struct mywork;
	int data;
};



irqreturn_t key_handler(int irq, void *dev)
{
	printk(KERN_INFO"key put down, irq: %d \n",irq);

	switch(irq)
	{
		case (IRQ_GPIO_B_START+30):
			printk(KERN_INFO"schedule work\n");
			schedule_work(&(((struct work_data*)dev)->mywork));
			break;
	}


	printk(KERN_INFO"workqueue goto ending\n");

	return IRQ_HANDLED;
}

void mywork_handler(struct work_struct *work)
{
	int cnt = 10;
	struct work_data *pdata;

	printk(KERN_INFO"mywork_Handler starting...\n");

	pdata = container_of(work,struct work_data,mywork);

	printk(KERN_INFO"data: %d \n",pdata->data);

	
	while(cnt--)
	{
		printk(KERN_INFO"looping......\n");
		mdelay(500);
	}

	return ;
}

int workque_init(void)
{
	int ret;

	wkdata.data = 100;
	INIT_WORK(&wkdata.mywork,mywork_handler);
	
	printk(KERN_WARNING"bound workque init\n");

	ret = request_irq((IRQ_GPIO_B_START+30),key_handler,IRQF_TRIGGER_FALLING,"key-intr",&wkdata);
	if(ret < 0)
	{
		printk(KERN_INFO"request irq failed\n");
		return -1;
	}

	
	return 0;
}

void workque_exit(void)
{
	printk(KERN_WARNING"bound workque exit\n");

	free_irq((IRQ_GPIO_B_START+30),NULL);
	cancel_work_sync(&wkdata.mywork);
	
	return ;
}


module_init(workque_init);

module_exit(workque_exit);

MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code, intr part");
MODULE_VERSION("1.0");
