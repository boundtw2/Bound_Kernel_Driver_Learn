#include <linux/init.h>
#include <linux/kernel.h>


int kModule_init(void)
{
	printk(KERN_WARNING"bound kmoudule init\n");
	
	return 0;
}

void kModule_exit(void)
{
	printk(KERN_WARNING"bound kmoudule exit\n");
	
	return ;
}


module_init(kModule_init);

module_exit(kModule_exit);

