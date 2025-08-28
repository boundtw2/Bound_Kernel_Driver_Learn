#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>
#include <mach/s5p6818_irq.h>
#include <cfg_type.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/wait.h>

static struct task_struct *kthr_task;
static DECLARE_WAIT_QUEUE_HEAD(kthr_wq);
static int kthr_flag = 0; // 标志位，控制线程工作

irqreturn_t key_handler(int irq, void *dev)
{
    printk(KERN_INFO "key put down, irq: %d\n", irq);

    switch (irq)
    {
        case (IRQ_GPIO_B_START+30):
            printk(KERN_INFO "goto kthread\n");
            kthr_flag = 1; // 设置标志
            wake_up_interruptible(&kthr_wq); // 唤醒线程
            break;
    }

    return IRQ_HANDLED;
}

int kthread_handler(void *data)
{
    while (!kthread_should_stop()) {
        // 线程在此休眠，直到标志为1或线程需要退出
        wait_event_interruptible(kthr_wq, kthr_flag || kthread_should_stop());
        if (kthread_should_stop())
            break;
        if (kthr_flag) {
            kthr_flag = 0; // 清除标志
            printk(KERN_INFO "into kthread handler\n");
            // 你的处理逻辑写在这里
        }
    }
    return 0;
}

static int __init KThr_init(void)
{
    int ret;

    printk(KERN_WARNING "bound KThread init\n");

    ret = request_irq((IRQ_GPIO_B_START+30), key_handler,
                      IRQF_TRIGGER_FALLING, "key-intr", NULL);
    if (ret < 0) {
        printk(KERN_INFO "request irq failed\n");
        return -1;
    }

    // 用 kthread_run 也可以，这里用 create+wake_up_process，线程自己会阻塞，不会马上执行业务逻辑
    kthr_task = kthread_create(kthread_handler, NULL, "kthread_bound");
    if (IS_ERR(kthr_task)) {
        printk(KERN_INFO "create kthread fail\n");
        free_irq((IRQ_GPIO_B_START+30), NULL);
        return -1;
    }
    wake_up_process(kthr_task); // 让线程启动，进入wait_event阻塞

    return 0;
}

static void __exit KThr_exit(void)
{
    printk(KERN_WARNING "bound workque exit\n");

    free_irq((IRQ_GPIO_B_START+30), NULL);
    if (kthr_task)
        kthread_stop(kthr_task);
}

module_init(KThr_init);
module_exit(KThr_exit);

MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code, intr part");
MODULE_VERSION("1.0");
