#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/timer.h>

struct timer_list myTimer;

void myTimer_handler(unsigned long data)
{
	printk(KERN_INFO"timer data: %lu\n", data);
	
	// Reschedule the timer
	mod_timer(&myTimer, jiffies + 2*HZ);
	// myTimer.expires = jiffies + 2*HZ;
	// add_timer(&myTimer);

	printk(KERN_INFO"timer rescheduled\n");
}

static int __init kTimer_init(void)
{
	printk(KERN_WARNING"bound kTimer init\n");

	init_timer(&myTimer);

	myTimer.expires = jiffies + 2*HZ;
	myTimer.data = 88;
	myTimer.function = myTimer_handler;

	add_timer(&myTimer);
	return 0;
}

static void __exit kTimer_exit(void)
{
	
	printk(KERN_WARNING"bound kTimer exit\n");
	del_timer(&myTimer);

	return ;
}


module_init(kTimer_init);

module_exit(kTimer_exit);

MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code, tasklet part");
MODULE_VERSION("1.0");