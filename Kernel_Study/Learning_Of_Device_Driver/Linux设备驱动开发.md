# Linux设备驱动开发









## 内核模块开发

### 内核模块介绍

1. 可以依赖内核库编译出来独立存在任意目录下的可执行文件
2. 平时主要是存放在用户空间的文件系统指定目录
3. 使用时要加载到内核空间中使用，不可以在用户空间直接执行
4. 是一个迷你版本的kernel，组成结构与kernel一致

### 静态开发方式

将驱动代码直接集成到内核镜像中，随系统启动自动加载，适用于核心功能（如存储、网络驱动）

特点：

	1. 开发周期长，且开发起来很麻烦
	1. 会伴随整个系统存在，使用效率比较低，占用内存
	1. 需要用到源码来开发驱动，如不开放，还要厂家代为编译

### 动态开发方式

以独立模块（`.ko` 文件）形式存在，通过 `insmod`/`rmmod` 动态管理

特点：

	1. 开发简单，开发周期短且比较方便
	1. 动态加载使用，不用去掉，可以使得核心的kernel较为精简，运行效率高

## 内核模块开发

### 示例代码

#### 模块代码

```c++
#include <linux/init.h>
#include <linux/kernel.h>


static int __init kModule_init(void)
{
	printk(KERN_WARNING"bound kmoudule init\n");
	
	return 0;
}

static void __exit kModule_exit(void)
{
	printk(KERN_WARNING"bound kmoudule exit\n");
	
	return ;
}


module_init(kModule_init);

module_exit(kModule_exit);
```

#### 新建Makefile

```makefile
obj-m += kmodule.o

KERNEL_DIR = /opt/kernel-build/kernel-3.4.39/
GCC_TOOL = /usr/local/arm/arm-eabi-4.8/bin/arm-eabi-

all:
	make modules -C $(KERNEL_DIR) M=`pwd` CROSS_COMPILE=$(GCC_TOOL)

clean:
	make modules clean -C $(KERNEL_DIR) M=`pwd` CROSS_COMPILE=$(GCC_TOOL)
```

#### 编译下载

```
make 编译
将.ko放到板子上
```

#### 加载删除模块

```
lsmod #查看现在有的驱动
insmod kmodule.ko #将编译得到的驱动文件运行到内核
rmmod kmodule #将运行的驱动删除
```

## 内核模块进阶

### 入口与出口

```c++
#include <linux/init.h>

module_init(kModule_init);

module_exit(kModule_exit);
```

module_init 用于声明一个内核模块的入口函数;一个普通函数，如果声明为入口函数，则在内核模块被 insmod 时自动被调用,主要提供给用户在插入模块之前做一些初始化的工作。
module_exit 用于声明一个内核模块的出口函数;一个普通函数，如果声明为出口函数，则在内核模块被 rmmod 时自动被调用,主要提供给用户在删除模块之前做一些恢复性的工作。

### 分析

```
typedef int (*initcall_t)(void);

#define module_init(initfn)					\
	static inline initcall_t __inittest(void)		\
	{ return initfn; }					\
	int init_module(void) __attribute__((alias(#initfn)));
```

这里定义了一个宏，即module_init，执行两部分代码

第一部分将代码进行内联，返回传入的`initfn`，将其转化为定义的`initcall_t`，即一个返回值为int (void)的函数指针

第二部分让 `init_module` 变成 `initfn` 的别名，调用 `init_module()` 就是调用 `initfn()`

模块通insmod，会做一系列动作，并且在最后一定会调用 init_module 函数，所以如果想要在 insmod 时调用这个函数，就一定要改名为 init_module

### tips-反编译

用反编译工具可以看到在编译后的.o文件中只有init_module和cleanup_module

```bash
root@bound-virtual-machine:/home/share/Kernel_Study/DD_Init_S# /usr/local/arm/arm-eabi-4.8/bin/arm-eabi-objdump -d kmodule.o

kmodule.o:     file format elf32-littlearm


Disassembly of section .text:

00000000 <init_module>:
   0:	e3000000 	movw	r0, #0
   4:	e92d4008 	push	{r3, lr}
   8:	e3400000 	movt	r0, #0
   c:	ebfffffe 	bl	0 <printk>
  10:	e3a00000 	mov	r0, #0
  14:	e8bd8008 	pop	{r3, pc}

00000018 <cleanup_module>:
  18:	e3000000 	movw	r0, #0
  1c:	e3400000 	movt	r0, #0
  20:	eafffffe 	b	0 <printk>

```

### __init/exit宏调用

```
__init = __section(.init.text)
```

用于声明后面的标号链接到.init.text 段，让使用 `__init` 修饰的函数或变量被放到内核镜像中的 `.init.text` 这个特殊段区

.init.text 段定义的物理意义是:
如果一段程序被链接到这个段区域,并且是定义在内核模块里面，则这段程序会在初始化阶段存在(即insmod 期间)，如果初始化完毕后，则这个段里面的所有内容都将会自动被释放掉。

### printk函数

printf函数，用于将信号导出到标准输出屏幕，给用户看，实现调试跟踪。

printk函数，是在内核内部使用，主要用于内核内部信息的记录(记录到 kmsg 文本)，还可以用于打印跟踪内核信息到屏幕显示。

格式:
printk(记录级别 要记录的信息)	比如:printk(KERN INFO “hello world\n”);

记录级别定义：

```c++
#define KERN_EMERG	"<0>"	/* system is unusable			*/
#define KERN_ALERT	"<1>"	/* action must be taken immediately	*/
#define KERN_CRIT	"<2>"	/* critical conditions			*/
#define KERN_ERR	"<3>"	/* error conditions			*/
#define KERN_WARNING	"<4>"	/* warning conditions			*/
#define KERN_NOTICE	"<5>"	/* normal but significant condition	*/
#define KERN_INFO	"<6>"	/* informational			*/
#define KERN_DEBUG	"<7>"	/* debug-level messages			*/
```

0级是最高，用于最重要的内核执行时记录的信息。7级是最低，主要用于标记用户的调试信息。

杨工说引入记录级别，为了后面从**`/proc/kmsg`**中快速查询相应记录信息。

ai说的查看方式：

| 查看方式     | 适用场景            | 命令                        |
| ------------ | ------------------- | --------------------------- |
| `dmesg`      | 绝大多数 Linux 系统 | `dmesg`                     |
| `/var/log/`  | 传统日志系统        | `tail -f /var/log/kern.log` |
| `journalctl` | 使用 systemd 的系统 | `journalctl -k`             |
| 串口/控制台  | 嵌入式、开发板      | 直接在串口工具里看          |

**`dmesg`**最常用的命令，会显示自系统启动以来的所有内核日志

printk可以将信息显示打印到屏幕，给用户看到，但是是有条件的，要求输出的记录级别要大于打印的参数（可以在系统中进行设置）

```c++
/* printk's without a loglevel use this.. */
#define DEFAULT_MESSAGE_LOGLEVEL CONFIG_DEFAULT_MESSAGE_LOGLEVEL

/* We show everything that is MORE important than this.. */
#define MINIMUM_CONSOLE_LOGLEVEL 1 /* Minimum loglevel we let people use */
#define DEFAULT_CONSOLE_LOGLEVEL 7 /* anything MORE serious than KERN_DEBUG */
```

- **DEFAULT_MESSAGE_LOGLEVEL**：`printk` 未指定级别时的默认值
- **MINIMUM_CONSOLE_LOGLEVEL**：控制台日志级别的最严格限制（安全下限）
- **DEFAULT_CONSOLE_LOGLEVEL**：系统启动时控制台的默认显示级别

### 信息宏（模块参数）使用

#include <linux/module.h>

定义了很多用于加载信息的宏，即信息宏

```c++
MODULE_AUTHOR("bound 13826168298 <113660120@qq.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Learning of Linux Driver Code");
MODULE_VERSION("1.0");
```

实现原理：

在内核模块中，可以使用信息宏，添加指定的信息，并且在编译时，会编译链接到.modinfo段里保存。使用者可以通过以下命令获取并显示出所有开发者添加进来的信息。

```
modinfo xxx.ko
```

### 多文件编译

将一个工程中，多个文件的程序编译成一个整体KO

Makefile语法：

```
obj-m += 多目标文件名称.o
多目标文件名称-y := 各个独立文件.o
```



```
obj-m += kmodule-kaddsub.o
kmodule-kaddsub-y := kmodule.o kaddsub.o
```

### 符号导出

在内核中，原来自带的函数以及后面通过内核模块插入的函数代码，如果想让别的函数模块可以调用到，那么这个函数一定做符号导出的声明。

如果想查看当前系统中，可以供用户调用的符号，则可以通过查符号导出表:有两种，静态符号导出表，动态符号导出表(静态符号导出表 +启动后动态插入的符号)

动态符号导出表： cat/proc/kallsyms 文件

符号导出格式：

```
#include <linux/module.h>

EXPORT_SYMBOL(标号); //这里的标号就是你要符号导出函数的函数名
```

## IO内存管理

### 内存相关概念

在嵌入式系统中，芯片（通常指主控芯片/CPU/MCU）与硬盘、存储器之间的连接和通信方式确实有很大不同。让我详细说明：

####  **芯片与存储器（RAM）的连接**

连接方式：

- **直接并行总线连接**：通过地址总线、数据总线和控制总线直连
- **专用内存控制器**：现代芯片内置DDR控制器，直接管理内存访问

通信特点：

```
CPU ←→ Memory Controller ←→ DDR/SRAM
     （内置）              （并行总线）
```

- **速度极快**：DDR4可达3200MHz，延迟极低（纳秒级）
- **随机访问**：可以直接寻址到任意地址
- **实时性**：CPU可以直接执行存储器中的代码
- **总线宽度**：通常32位或64位并行数据总线

#### **芯片与硬盘（Flash存储）的连接**

连接方式：

根据不同类型的Flash存储，连接方式不同：

**SPI NOR Flash**

```
CPU ←→ SPI Controller ←→ SPI NOR Flash
                       （4线串行：CS/CLK/MOSI/MISO）
```

**NAND Flash**

```
CPU ←→ NAND Controller ←→ NAND Flash
                        （8/16位并行 + 控制信号）
```

**eMMC/SD卡**

```
CPU ←→ SDIO/MMC Controller ←→ eMMC
                            （4/8线串行 + 控制线）
```

**SATA SSD**

```
CPU ←→ SATA Controller ←→ SATA SSD
                        （串行差分信号）
```

通信特点：

- **速度较慢**：相比RAM慢很多，需要擦写时间
- **块访问**：以页面或扇区为单位读写
- **间接访问**：需要通过文件系统或驱动程序
- **串行传输**：多数采用串行接口，引脚数少

#### **具体技术对比表**

| 项目         | RAM连接        | Flash存储连接  |
| ------------ | -------------- | -------------- |
| **接口类型** | 并行总线       | 串行接口为主   |
| **访问速度** | 极快（GB/s级） | 较慢（MB/s级） |
| **访问方式** | 随机访问       | 块/页访问      |
| **延迟**     | 纳秒级         | 微秒到毫秒级   |
| **控制器**   | 内置DDR控制器  | 专用接口控制器 |
| **引脚数量** | 多（20-200+）  | 少（4-16根）   |

#### **实际案例分析**

**ARM Cortex-A系列处理器**：

```
┌─────────────────┐
│   ARM Core      │
├─────────────────┤
│ DDR Controller  │ ←→ DDR3/4 RAM (并行总线)
│ SPI Controller  │ ←→ SPI NOR Flash (4线串行)
│ NAND Controller │ ←→ NAND Flash (并行)
│ SDIO Controller │ ←→ eMMC (串行)
└─────────────────┘
```

**单片机STM32**：

```
┌─────────────────┐
│ Cortex-M Core   │
├─────────────────┤
│ SRAM (内置)     │ ←→ 直接连接
│ Flash (内置)    │ ←→ 直接连接
│ SPI Peripheral  │ ←→ 外部SPI Flash
│ SDIO Peripheral │ ←→ SD卡
└─────────────────┘
```

####  **为什么这样设计？**

**RAM需要高速并行**：

- CPU需要实时执行代码，要求极低延迟
- 大数据量传输需要高带宽
- 随机访问模式适合并行总线

**Flash适合串行接口**：

- 主要用于存储，对速度要求不如RAM严格
- 串行接口节省引脚，降低成本
- 块访问模式适合文件存储

####  **新趋势**

- **高速存储**：NVMe SSD采用PCIe接口，提升存储速度
- **统一内存**：某些嵌入式系统开始采用统一内存架构
- **3D XPoint**：新型存储技术试图填补RAM和Flash之间的性能差距

这种分层设计既保证了系统性能，又控制了成本，是现代嵌入式系统的核心架构原理。

我来详细为您阐述嵌入式Linux中的内存空间和IO空间概念。

## 内存空间与IO空间

处理器内存空间的本质：它是一个统一的寻址空间，通过地址总线的宽度限制了最大容量，并且在这个空间内合理分配IO内存空间 + 系统RAM空间。

### 内存空间（Memory Space）

#### 定义和特点
内存空间是CPU可直接访问的地址空间，用于存储程序代码、数据和缓存。在嵌入式Linux系统中，内存空间具有以下特点：

**1. 统一寻址**

- CPU使用相同的指令和寻址方式访问内存
- 支持字节、字、双字等不同粒度的访问
- 可以执行读写操作

**2. 物理内存布局**
```
典型的ARM嵌入式系统内存映射：
0x00000000 - 0x0FFFFFFF: Boot ROM/Flash
0x10000000 - 0x1FFFFFFF: 内部SRAM
0x20000000 - 0x2FFFFFFF: 外部SDRAM
0x30000000 - 0x3FFFFFFF: 外设寄存器
```

**3. 虚拟内存管理**
- MMU（内存管理单元）负责地址转换
- 支持内存保护和虚拟地址映射
- 实现进程间内存隔离

### IO空间（I/O Space）

#### 定义和特点
IO空间是专门用于访问外设寄存器和控制器的地址空间。在嵌入式系统中有两种主要实现方式：

**1. 内存映射IO（MMIO - Memory Mapped I/O）**

```c
// 典型的MMIO访问示例
#define GPIO_BASE    0x20200000
#define GPIO_FSEL0   (GPIO_BASE + 0x00)
#define GPIO_SET0    (GPIO_BASE + 0x1C)

volatile uint32_t *gpio_fsel = (uint32_t *)GPIO_FSEL0;
*gpio_fsel |= (1 << 3);  // 设置GPIO引脚模式
```

**2. 端口映射IO（Port I/O）**

```c
// x86架构中的端口IO示例
outb(0x3F8, 'H');  // 向串口发送字符
char data = inb(0x3F8);  // 从串口读取字符
```

### 两者的主要区别

#### 1. 访问方式
**内存空间：**

- 使用标准内存访问指令（LDR/STR等）
- 支持所有内存寻址模式
- 可以使用指针直接访问

**IO空间：**

- 使用专门的IO访问函数
- 在Linux内核中通过ioremap()映射
- 需要特殊的访问权限

#### 2. 缓存特性
**内存空间：**

- 通常支持缓存加速
- 可以进行预取和写缓冲
- 访问顺序可能被优化

**IO空间：**

- 通常不可缓存
- 必须保证访问顺序
- 每次访问都直达硬件

#### 3. Linux内核中的处理

**内存空间访问：**

```c
// 分配和映射内存
void *virt_addr = kmalloc(size, GFP_KERNEL);
void *phys_addr = virt_to_phys(virt_addr);
```

**IO空间访问：**
```c
// 映射IO内存
void __iomem *io_base = ioremap(phys_addr, size);

// 安全的IO访问
uint32_t value = readl(io_base + offset);
writel(data, io_base + offset);

// 释放映射
iounmap(io_base);
```

### MMU的主要作用

1. **虚拟地址到物理地址的转换 (Address Translation):**

   - **概念:** 在现代操作系统（如Linux）中，每个进程都运行在自己的独立*虚拟地址空间*中。这意味着每个进程“认为”自己拥有整个可用的内存范围（例如，从0到2^32或2^64，取决于CPU架构）。然而，实际的*物理内存*（RAM）是有限的，并且由所有进程和操作系统内核共享。
   - **MMU的角色:** MMU负责将CPU和软件使用的*虚拟地址*动态地转换成RAM芯片中的实际*物理地址*。这个转换过程对每次内存访问都会进行。
   - 重要性:
     - **进程隔离 (Process Isolation):** 每个进程拥有独立的虚拟地址空间，MMU确保一个进程不能意外或恶意地访问另一个进程的内存或内核的内存，从而提高了系统的稳定性和安全性。
     - **内存效率 (Memory Efficiency):** 允许系统更有效地使用物理内存。例如，程序中当前未使用的部分可能不会加载到物理RAM中，直到需要时才加载（称为“按需分页”）。

2. **内存保护 (Memory Protection):**

   - **概念:** MMU可以为内存的不同区域强制执行访问权限（例如，只读、可读写、可执行）。
   - **MMU的角色:** 在地址转换期间，MMU会检查这些权限。如果一个进程试图执行未经授权的访问（例如，写入一个只读内存区域，或执行数据区域的代码），MMU会产生一个硬件异常（通常称为“段错误”或“页错误”），操作系统内核会捕获这个异常并进行处理（通常是终止违规的进程）。
   - 重要性:
     - **防止损坏 (Prevents Corruption):** 保护操作系统内核免受用户进程的破坏，也保护用户进程之间互相干扰。
     - **安全性 (Security):** 有助于防止某些试图覆盖关键代码或数据的攻击。
     - **调试 (Debugging):** 有助于捕获常见的编程错误，如空指针解引用或缓冲区溢出。

3. **支持虚拟内存 (Support for Virtual Memory):**

   - **概念:** 虚拟内存是一种技术，它允许系统使用的内存看起来比实际物理内存更大。这是通过使用磁盘空间（通常称为*交换空间*或*swap space*）作为RAM的扩展来实现的。

   - **MMU的角色:**

     MMU与操作系统协同工作，管理进程虚拟地址空间的哪些部分当前在物理RAM中，哪些部分存储在磁盘上。

     - **分页 (Paging):** 内存被划分为固定大小的块，称为*页 (page)*（虚拟内存中）和*页帧 (page frame)*（物理内存中）。MMU使用由操作系统维护的*页表 (page tables)*来跟踪虚拟页和物理页帧之间的映射关系。

     - **缺页中断 (Page Fault):**

       如果进程试图访问一个当前不在物理RAM中的虚拟页（例如，它可能已被换出到磁盘，或者从未被加载过），MMU会产生一个缺页中断。操作系统内核会处理这个中断，通常步骤如下：

       1. 在物理RAM中找到一个空闲的页帧（或者选择一个当前占用的页帧并将其内容写回磁盘，以腾出空间）。
       2. 从磁盘加载所需的页到该页帧。
       3. 更新页表以反映新的映射关系。
       4. 重新执行导致缺页中断的指令。

   - 重要性:

     - **更大的地址空间 (Larger Address Space):** 允许运行比可用物理RAM更大的应用程序。
     - **高效的内存使用 (Efficient Memory Use):** 只有程序中活动的部分才需要驻留在RAM中。
     - **共享库 (Shared Libraries):** 允许多个进程在物理内存中共享库代码的单个副本，尽管每个进程都有自己对该库的虚拟地址。

**相关概念解析：**

- **虚拟地址 (Virtual Address - VA):** CPU和正在运行的程序使用的地址。
- **物理地址 (Physical Address - PA):** 硬件内存（RAM）中的实际地址。
- **页 (Page):** 虚拟内存的一个固定大小的块。常见大小有4KB、8KB等。
- **页帧 (Page Frame):** 物理内存的一个固定大小的块，大小与页对应。
- **页表 (Page Table)**:由操作系统维护并由MMU使用的数据结构（或一组结构），用于存储虚拟页到物理页帧的映射关系。页表中的每个条目（Page Table Entry - PTE）通常包含：
  - 物理页帧号。
  - 访问权限位（读、写、执行、用户模式/内核模式）。
  - 状态位（存在/不存在于内存、脏位（是否被修改过）、访问位（是否被访问过））。
- **快表 (Translation Lookaside Buffer - TLB):** MMU内部的一个小型、高速的缓存，用于存储最近使用过的虚拟地址到物理地址的转换。这显著加快了地址转换过程，因为访问主存（页表存储在主存中）比访问TLB慢得多。如果在TLB中找到转换（称为“TLB命中”），则快速获得物理地址。如果未找到（“TLB未命中”），MMU必须在主存中的页表中查找转换。
- **上下文切换 (Context Switch):** 当操作系统从一个进程切换到另一个进程运行时，MMU的上下文（例如，指向当前进程页表的指针）也必须切换。这确保了新运行的进程使用其自己独立的虚拟地址空间。

## Linux中断处理

**什么是硬件中断？**

想象一下你在专心工作（CPU 执行程序），突然门铃响了（硬件事件发生）。你不得不暂停手头的工作，去开门（处理中断），处理完开门这件事后，再回来继续之前的工作。

硬件中断就是这样一个机制：**外部或内部的硬件设备产生一个信号，通知 CPU 有紧急事件需要处理。CPU 会暂停当前正在执行的任务，转而去执行一个专门为该中断准备的处理程序（中断服务程序 - ISR），处理完毕后再返回到之前被暂停的任务继续执行。**

### **一、ARM 体系架构层面**

ARM 架构为中断处理提供了一套标准化的机制和规范。

1.  **异常模型 (Exception Model):**
    *   中断在 ARM 中被视为一种**异常 (Exception)**。ARM 定义了多种异常类型，如复位、未定义指令、软件中断 (SVC)、数据中止、预取中止，以及我们关注的 **IRQ (Interrupt Request)** 和 **FIQ (Fast Interrupt Request)**。
    *   **IRQ (中断请求):** 通用中断，数量较多，可屏蔽。
    *   **FIQ (快速中断请求):** 优先级通常高于 IRQ，设计用于需要极低延迟处理的事件（如高速数据传输）。FIQ 通常有自己独立的寄存器组，以减少上下文切换的开销。

2.  **中断向量表 (Vector Table):**
    *   当一个异常（包括中断）发生时，CPU 会自动跳转到内存中一个预定义地址区域，这个区域就是中断向量表。
    *   向量表中存放的是各个异常处理程序的入口地址（或者跳转到入口地址的指令）。CPU 根据中断类型查找对应的向量，然后跳转执行。

3.  **中断控制器接口 (GIC - Generic Interrupt Controller):**
    *   现代 ARM 处理器通常不直接处理成百上千个外设中断，而是依赖一个**通用中断控制器 (GIC)**。ARM 定义了 GIC 的标准架构（如 GICv2, GICv3, GICv4）。
    *   **GIC 的核心功能:**
        *   **接收中断:** 从各个外设接收中断请求。
        *   **优先级管理:** 为中断设置优先级，并根据优先级决定哪个中断先被处理。
        *   **中断分发:** 将最高优先级且已使能的中断信号发送给一个或多个 CPU 核心。
        *   **中断屏蔽/使能:** 可以屏蔽或使能特定的中断源。
    *   **GIC 中的中断类型:**
        *   **SGI (Software Generated Interrupts):** 软件产生的中断，主要用于多核间的通信。
        *   **PPI (Private Peripheral Interrupts):** 私有外设中断，特定于某个 CPU 核心的中断（如该核心的定时器）。
        *   **SPI (Shared Peripheral Interrupts):** 共享外设中断，来自系统中的共享外设（如 UART, I2C, GPIO 控制器等），可以路由到任何一个 CPU 核心。

4.  **中断处理流程 (简化版):**
    1.  硬件事件发生，外设向 GIC 发送中断信号。
    2.  GIC 根据优先级和使能状态，向某个 CPU 核心发送 IRQ 或 FIQ 信号。
    3.  CPU 核心检测到中断信号：
        *   保存当前程序状态（如 PC 指针、状态寄存器 CPSR/PSTATE）。
        *   切换到相应的异常模式/级别。
        *   （通常）屏蔽同级或低优先级中断。
        *   从 GIC 的 CPU 接口读取中断号 (Interrupt ID)，以识别是哪个具体的中断源。
        *   跳转到中断向量表，再跳转到对应的中断服务程序 (ISR)。
    4.  执行 ISR。
    5.  ISR 执行完毕后，向 GIC 发送中断结束信号 (EOI - End of Interrupt)。
    6.  CPU 恢复之前保存的程序状态，返回到被中断的程序继续执行。

5.  **中断屏蔽:**
    *   CPU 的状态寄存器 (CPSR/SPSR 或 PSTATE) 中有 `I` (IRQ) 和 `F` (FIQ) 位，可以全局屏蔽 IRQ 和 FIQ。
    *   GIC 也提供了更细粒度的屏蔽，可以针对每个中断源进行屏蔽。

### **二、SoC (System on Chip) 芯片层面**

SoC 层面是 ARM 架构和 GIC 规范的具体实现和集成。

1.  **集成:**
    *   SoC 将 ARM CPU 核心、GIC、各种外设（UART、SPI、I2C、DMA、定时器、GPIO 控制器等）、内存控制器以及它们之间的互联总线都集成在一块芯片上。

2.  **外设作为中断源:**
    *   SoC 上的几乎所有外设都可以配置为中断源。例如，UART 接收到数据、定时器超时、GPIO 引脚电平变化等都可以触发中断。

3.  **中断线的连接:**
    *   在 SoC 内部，这些外设的中断输出线会物理连接到 GIC 的输入引脚上。具体哪个外设连接到 GIC 的哪个 SPI 或 PPI 输入，是由 SoC 硬件设计决定的，并且会在该 SoC 的**数据手册 (Datasheet)** 中详细列出。
    *   例如，SoC 数据手册会告诉你 UART0 的中断连接到 GIC 的 SPI 编号 X，GPIOA 控制器的中断连接到 GIC 的 SPI 编号 Y。

4.  **GPIO 中断的特殊性:**
    *   一个 GPIO 控制器通常管理很多 GPIO 引脚（如 32 个或更多）。这些引脚通常不会每个都有单独的中断线连到 GIC。
    *   而是，整个 GPIO 控制器组（例如 GPIOA）共享一个或少数几个中断线连接到 GIC。当该组内任何一个配置为中断的引脚发生电平变化时，GPIO 控制器会向 GIC 发送一个中断信号。
    *   在 ISR 中，软件需要读取 GPIO 控制器的内部状态寄存器，才能确定是哪个具体的 GPIO 引脚触发了中断。

5.  **外设级中断控制:**
    *   除了 GIC 提供的中断使能/屏蔽外，每个外设通常也有自己的中断控制寄存器，用于：
        *   使能或禁止该外设产生中断。
        *   清除中断标志位（当中断被处理后）。
        *   配置中断触发方式（电平触发、边沿触发）。

6.  **时钟和电源管理:**
    *   外设和 GIC 都需要时钟才能工作。电源管理单元 (PMU) 可能会关闭不活动外设的时钟或电源，这会影响它们产生中断的能力。

**总结来说：**

*   **ARM 架构**层面提供了中断处理的**规范和核心机制**，如异常模型、向量表以及 GIC 标准。
*   **SoC 芯片**层面则是这些规范的**具体硬件实现和集成**，它决定了有哪些外设、这些外设如何连接到 GIC、以及每个外设内部的中断控制逻辑。

在开发驱动程序时，开发者需要同时理解 ARM 的通用中断机制和特定 SoC 的中断映射及外设配置细节（主要通过查阅 SoC 数据手册）。操作系统内核（如 Linux）的中断子系统会基于这些硬件机制，为驱动程序提供更高层次的中断请求、注册和处理接口。

### 中断使用

#### 中断注册与注销

申请使用IRQ中断

```c++
int __must_check request_irq(
    unsigned int irq,         // 要分配的中断线编号 (IRQ number)
    irq_handler_t handler,    // 中断发生时调用的主处理函数 (硬中断上下文)
    unsigned long irqflags,   // 中断类型标志
    const char *devname,      // 申请中断的设备名称 (ASCII字符串)
    void *dev_id              // 传递给处理函数的 cookie (通常是设备私有数据指针)
);
```

释放占用IRQ中断

```c++
void free_irq(
	unsigned int, 	// 要释放的中断线编号 (IRQ number)
	void *
);
```

linux系统中，针对每一种型号的芯片，都会对应一个封装接口

而对应的中断号，就封装在irqs.h中。比如s5p6818给到的内核代码中，irqs.h中则如下所示：

```c++
#include "s5p6818_irq.h"

#define NR_IRQS				IRQ_TOTAL_MAX_COUNT
```

而真正6818平台中断号封装的地方就在s5p6818_irq.h

### 中断号的计算

S5P6818 定义了一种分层的中断编号方案，这在嵌入式 Linux 系统中很常见。我们需要考虑几个层面：

1. **GIC (通用中断控制器) 中断 (硬件 IRQ):** 这些是进入 GIC 的原始中断线。
   - ID 0-15 通常是 SGI (软件产生的中断)。
   - ID 16-31 通常是 PPI (私有外设中断)，针对每个 CPU 核心。
   - ID 32 及以上是 SPI (共享外设中断)，用于各种外设。
2. **头文件中定义的“物理 (Physical)”中断号:** 这些似乎对应于 GIC 的 SPI 编号。请注意它们都带有 `+ 32`。
   - 例如，`IRQ_PHY_UART0 = (7 + 32)` 表示 UART0 使用 GIC SPI 编号 39。
   - 类似地，`IRQ_PHY_GPIOB = (54 + 32)`，结果是 **86**。这是**整个 GPIOB 控制器组 (bank)** 的 GIC 中断号。当 GPIOB 上任何一个使能的引脚产生中断时，GIC 的 86 号中断会被触发。
3. **Linux 内核 IRQ 号 (软件 IRQ):** Linux 内核会接收这些硬件 IRQ，并将它们映射到其内部的 IRQ 编号。这个头文件似乎正在为这些内核 IRQ 号定义一个特定的布局，特别是针对单个 GPIO 引脚。单个 GPIO 引脚通常没有自己专用的 GIC 中断线，而是通过其 GPIO 控制器共享一条线。

**计算 GPIOB30 的中断号：**

1. **GPIOB 控制器组的 GIC 中断号:** 如上所示： `#define IRQ_PHY_GPIOB (54 + 32)` 所以，整个 GPIOB 控制器的 GIC SPI 编号是 **86**。 当 GPIOB30 发生中断时，GIC 会发出 86 号中断信号。Linux 内核的 GIC 驱动程序会处理这个中断，然后 S5P6818 的 GPIO 驱动程序会接管。GPIO 驱动程序需要确定是 GPIOB 中的*哪个具体引脚*导致了中断，然后调用与该引脚的特定*软件 IRQ 号*相关联的正确处理程序。

2. **GPIOB30 的软件 IRQ 号:** 这就是 `IRQ_GPIO_...` 这些宏定义起作用的地方。它们为单个 GPIO 引脚定义了一个连续的软件 IRQ 号块。

   - `#define IRQ_PHY_MAX_COUNT (74 + 32)` `IRQ_PHY_MAX_COUNT = 106`

   - `#define IRQ_GPIO_START IRQ_PHY_MAX_COUNT` 所以，`IRQ_GPIO_START = 106`。这是在这个软件方案中，单个 GPIO 引脚 IRQ 编号的起始基数。

   - `#define IRQ_GPIO_B_START (IRQ_GPIO_START + PAD_GPIO_B)` 我们需要找到 `PAD_GPIO_B` 的定义。这个宏在您提供的片段中没有定义，但它代表 GPIO 组 B 在 GPIO 中断号空间内的偏移量。 观察规律： `#define IRQ_GPIO_END (IRQ_GPIO_START + 32 * 5) // Group: A,B,C,D,E` 这强烈暗示有 5 个 GPIO 组 (A, B, C, D, E)，并且在这个方案中每组分配了 32 个中断号。 因此，非常有可能：

     - `PAD_GPIO_A` 对应偏移量 `0 * 32 = 0`
     - `PAD_GPIO_B` 对应偏移量 `1 * 32 = 32`
     - `PAD_GPIO_C` 对应偏移量 `2 * 32 = 64`
     - 以此类推。

     假设 `PAD_GPIO_B = 32` (这是一种非常标准的定义方式)： `IRQ_GPIO_B_START = IRQ_GPIO_START + 32` `IRQ_GPIO_B_START = 106 + 32` `IRQ_GPIO_B_START = 138`

   - 现在，要获取 GPIOB**30** 的具体中断号，您需要将引脚索引 (30) 添加到 B 组中断号的起始值上： GPIOB30 的中断号 = `IRQ_GPIO_B_START + 30` GPIOB30 的中断号 = `138 + 30` GPIOB30 的中断号 = **168**

**GPIOB30 软件 IRQ 号的计算总结：**

1. `IRQ_PHY_MAX_COUNT = (74 + 32) = 106`
2. `IRQ_GPIO_START = IRQ_PHY_MAX_COUNT = 106`
3. 假设 `PAD_GPIO_B = 32` (因为它是第二组，并且每组似乎分配了 32 个 IRQ)。
4. `IRQ_GPIO_B_START = IRQ_GPIO_START + PAD_GPIO_B = 106 + 32 = 138`
5. GPIOB**30** 的 IRQ 号 = `IRQ_GPIO_B_START + 30 = 138 + 30 = 168`

### 主处理函数

形如：

```c++
irqreturn_t my_handler(int irq, void *dev_id)
{ 
    /* 这里处理逻辑，比如唤醒进程、设置标志位等 */
    
    return IRQ_HANDLED; 
}
```

### 中断类型标志

在interrupt.h中定义了各种各样的宏，这些宏**定义了 Linux 内核中断注册（request_irq）时的各种中断标志（IRQ flags）**，用于描述**中断触发方式**和**中断处理行为**。
它们的意义如下：

------

#### IRQF_TRIGGER_* 相关宏

这些宏用于**描述中断的硬件触发方式**，告诉内核**什么样的信号变化会触发中断**，常用于GPIO等外部中断。

| 宏名                 | 含义说明                                                 |
| -------------------- | -------------------------------------------------------- |
| IRQF_TRIGGER_NONE    | 不指定触发类型，使用硬件或固件默认配置（通常是上电状态） |
| IRQF_TRIGGER_RISING  | 上升沿触发（低→高电平变化时触发）                        |
| IRQF_TRIGGER_FALLING | 下降沿触发（高→低电平变化时触发）                        |
| IRQF_TRIGGER_HIGH    | 高电平触发，只要处于高电平就触发                         |
| IRQF_TRIGGER_LOW     | 低电平触发，只要处于低电平就触发                         |
| IRQF_TRIGGER_MASK    | 用于屏蔽/过滤所有触发类型相关的宏                        |
| IRQF_TRIGGER_PROBE   | 用于探测中断触发类型（通常由内核自动探测时用）           |

**举例**：

- 按键通常用 `IRQF_TRIGGER_FALLING`（按下时高->低）
- 红外接收可能用 `IRQF_TRIGGER_RISING` 或 `IRQF_TRIGGER_LOW`

------

#### 其他 IRQF_* 标志

这些宏**控制内核处理中断的行为**，影响中断处理的方式或特殊用途。

| 宏名               | 含义说明                                                     |
| ------------------ | ------------------------------------------------------------ |
| IRQF_DISABLED      | **已废弃**。历史上用于中断处理期间关闭本CPU中断。现在无作用。 |
| IRQF_SAMPLE_RANDOM | 该中断用于熵池，给内核随机数生成器提供熵（如鼠标或键盘中断） |
| IRQF_SHARED        | 允许该中断号被多个设备共享（如多网卡共用一个中断线）         |
| IRQF_PROBE_SHARED  | 探测共享中断时用，允许在probe阶段出现中断号冲突              |
| __IRQF_TIMER       | 内部标志，标记这是一个定时器中断                             |
| IRQF_PERCPU        | 该中断为每个CPU独立分配（即每个CPU有自己的中断处理）         |
| IRQF_NOBALANCING   | 不参与中断负载均衡（即不会被IRQ balance迁移到其它CPU）       |
| IRQF_IRQPOLL       | 该中断用于轮询机制时（主要用于磁盘IO等场景）                 |
| IRQF_ONESHOT       | 用于线程化中断处理，hardirq处理后不会立刻重新使能中断（直到threaded handler处理完才使能） |
| IRQF_NO_SUSPEND    | 系统suspend时不屏蔽此中断                                    |
| IRQF_FORCE_RESUME  | resume阶段强制使能此中断，即使IRQF_NO_SUSPEND已设置          |
| IRQF_NO_THREAD     | 不允许线程化此中断（即不能转为threaded irq handler）         |
| IRQF_EARLY_RESUME  | 在syscore恢复阶段提前恢复此中断，而不是设备恢复阶段          |

### dev的作用

1. 用作“上下文／私有数据指针”
   — 在你写的中断处理函数里，内核会将你传进去的 `dev_id` 再原样传给 handler：

   ```c++
   irqreturn_t my_handler(int irq, void *dev_id)
   {
       struct my_device *dev = dev_id;
       /* —— 你就可以在这里通过 dev 访问设备私有数据 —— */
       …
       return IRQ_HANDLED;
   }
   ```

   — 这样你就能在中断上下文中拿到对应的设备结构体、状态标志、缓冲区指针等等。

2. 用于中断“共享”和“匹配”
   — 当你用 `IRQF_SHARED` 注册一条可共享的 IRQ，内核会维护一个 handler 链表。每条 handler 都要绑定一个唯一的 `dev_id`；
   — 在触发时，内核轮流调用这个 IRQ 号下所有注册者的 handler，传入各自的 `dev_id`；
   — 在你卸载／释放中断（`free_irq(irq, dev_id)`）时，内核也会用 `dev_id` 去找到并移除对应的 handler。

简单举个典型例子──假设你有一个设备结构体 `struct foo_dev`：

```c++
struct foo_dev {
    /* … 设备寄存器基址、缓冲区、状态标志等 … */
};

static irqreturn_t foo_irq_handler(int irq, void *dev_id)
{
    struct foo_dev *f = dev_id;
    /* 在这里用 f 去读寄存器、clear 中断、唤醒线程等 */
    return IRQ_HANDLED;
}

static int foo_probe(struct platform_device *pdev)
{
    struct foo_dev *f;
    int irq, ret;

    /* … 分配并初始化 f … */
    irq = platform_get_irq(pdev, 0);
    ret = request_irq(irq,
                      foo_irq_handler,
                      IRQF_SHARED | IRQF_TRIGGER_FALLING,
                      "foo-device",
                      f);    /* ← 把 f 传进去 */
    if (ret)
        /* 失败处理 */;
    return 0;
}

static int foo_remove(struct platform_device *pdev)
{
    struct foo_dev *f = platform_get_drvdata(pdev);
    free_irq(irq, f);  /* ← 用同样的 dev_id 去释放 */
    /* … 其它清理 … */
    return 0;
}
```

### 中断开关

```c++
void enable_irq(unsigned int irq);
void disable_irq(unsigned int irq);
```

### 相关的中断特性

**a. 响应中断时会独占 CPU**

- **解释**：在单核 CPU 上，响应硬件中断时，内核会立即打断正在执行的进程（无论是用户空间还是内核空间），转而去执行对应的中断处理程序（中断服务例程/ISR）。
  - 在 ISR 执行期间，当前 CPU 不会切换到其他任务，确实“独占”CPU。
  - **但注意**：中断处理程序一般允许更高优先级（嵌套）中断再次打断自己（除非明确禁用更高优先级中断）。
  - **结论**：ISR 虽然独占 CPU，但不是绝对屏蔽所有中断，只是抢占了普通任务的执行权。

------

**b. 中断程序设计不宜使用长时间停留语句**

- **解释**：中断服务例程应尽量简短高效，避免循环等待、长时间运算、复杂逻辑等。

  - ISR 执行期间正常进程无法运行，时间久了会引起系统响应变慢、调度延迟增大。
  - 影响其他中断的实时性，甚至造成“中断丢失”或“死机”。

  - **将复杂耗时的操作放到“下半部”**（如tasklet、workqueue、softirq、threaded irq等）执行，ISR 只完成最小必要处理（如清中断标志、数据搬运、唤醒线程等）。

------

**c. 中断程序与应用程序属于两个不同区段的内容，即分时运行及隔离空间(传参)**

- 中断程序（ISR）属于内核态，应用程序属于用户态，二者地址空间隔离，不能直接访问彼此的数据结构。

  - 中断处理是“异步”的，并不是在应用程序控制下直接被调用。

  - **传参方式**：应用层要与 ISR 交互，通常通过环形缓冲区、flag、等待队列、唤醒信号等方式间接“通信”。
  - **误区说明**：ISR 不能直接调用用户空间函数，不能直接操作用户空间指针。
  - **安全性**：这种分离保证了系统安全和稳定，防止 ISR 破坏用户数据，也防止应用恶意篡改内核。

------

**d. 中断程序不要使用会休眠的接口函数，比如 ssleep(3);**

- **解释**：ISR 不能使用任何可能导致当前任务休眠（sleep/block）的接口，包括 `ssleep()`、`msleep()`、`schedule()`, `mutex_lock()`（如果可能阻塞）等。

  - ISR 代码执行在不可中断上下文（atomic context），没有进程调度栈，不能被调度器切换出去，否则会内核异常（BUG/warning）。

  - **如果需要“等待”或“延迟”操作，应该设计成中断下半部或者内核线程完成。**
  - 在 ISR 里只能用原子操作、自旋锁等，不要用任何会阻塞的同步机制。

### 中断上、底半部

中断响应时CPU会被独占，所以要求中断程序不要设计长延时代码；而用户若要实现的功能中要长时间地进行停留处理呢？

于是乎就引出了将中断处理切成上半部与底半部两部分处理。

中断上半部功能：配置中断，启用底半部

中断底半部功能：实现具体的功能业务

底半部实现机制：

​	1） 小任务处理机制： tasklet

​	2） 工作队列处理机制：worker

​	3） 内核线程处理机制： kthread



### 小任务处理机制

Tasklet是Linux内核中实现的一种**软中断下半部（bottom half, BH）**机制，用于推迟中断处理程序中耗时的工作。tasklet具有轻量、低延迟、无须创建内核线程、可用于并发等优点。下面从原理、结构、使用方法和与其他机制的对比等方面详细解析tasklet。

---

1. 背景和作用

当硬件产生中断时，内核的中断处理程序（ISR）分为两部分：

- **上半部（top half）**：快速完成与硬件相关的操作，尽量短小，通常只设置标志或缓存数据。
- **下半部（bottom half）**：处理耗时操作，如数据复制、协议栈处理等。

tasklet就是一种下半部机制。

---

2. tasklet的结构体

内核中tasklet由`struct tasklet_struct`描述，定义在`<linux/interrupt.h>`：

```c
struct tasklet_struct {
    struct tasklet_struct *next;
    unsigned long state;
    atomic_t count;
    void (*func)(unsigned long);
    unsigned long data;
};
```

- `func`：tasklet的处理函数。
- `data`：传递给处理函数的参数。
- `count`：用于使能/禁止tasklet（为0可调度，为非0禁止）。
- `state`：状态标志，常用有TASKLET_STATE_SCHED、TASKLET_STATE_RUN。
- `next`：用于链接多个tasklet。

---

3. tasklet的使用方法

3.1 定义和初始化

初始化tasklet有两种方式：

静态定义

```c
void my_tasklet_func(unsigned long data) {
    // 处理代码
}

DECLARE_TASKLET(my_tasklet, my_tasklet_func, data);
```

动态初始化

```c
struct tasklet_struct my_tasklet;
tasklet_init(&my_tasklet, my_tasklet_func, data);
```

3.2 调度tasklet

```c
tasklet_schedule(&my_tasklet);
```

这会将tasklet插入到CPU的tasklet队列中，稍后由内核调度执行。

3.3 使能/禁止tasklet

```c
tasklet_disable(&my_tasklet); // 禁止
tasklet_enable(&my_tasklet);  // 使能
tasklet_kill(&my_tasklet);    // 等待tasklet执行完毕并释放资源
```

---

4. tasklet的执行时机和上下文

- tasklet由软中断（softirq）机制驱动，具体由`ksoftirqd`内核线程或在中断返回时执行。
- 运行在软中断上下文，不能睡眠、不能阻塞、不能访问用户空间。

---

5. tasklet的特性

- **同一类型tasklet在同一时刻只会在一个CPU上运行（同一个tasklet不会并发），但不同tasklet可在不同CPU上并发运行。**
- 不能保证tasklet的实时性，但通常延迟较低。
- 不能睡眠，适合短小的延后处理。

---

6. 与其他下半部机制的对比

| 机制      | 并发性        | 能否睡眠 | 适用场景             |
| --------- | ------------- | -------- | -------------------- |
| tasklet   | 不同CPU可并发 | 否       | 轻量、无阻塞操作     |
| workqueue | 可并发        | 可以     | 需要睡眠的复杂操作   |
| timer     | 定时执行      | 否       | 延时触发的轻量任务   |
| softirq   | 并发、底层    | 否       | 网络、块设备等高性能 |

---

7. 典型使用场景

- 网络驱动数据包处理
- USB等设备驱动的事件延后处理
- 需要在中断后尽快完成但不适合在中断上半部完成的工作

---

8. 示例代码

```c
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
#include <linux/interrupt.h>

void example_tasklet_func(unsigned long data) {
    printk(KERN_INFO "tasklet executed with data: %lu\n", data);
}

struct tasklet_struct example_tasklet;

static int __init my_init(void) {
    tasklet_init(&example_tasklet, example_tasklet_func, 123);
    tasklet_schedule(&example_tasklet);
    return 0;
}

static void __exit my_exit(void) {
    tasklet_kill(&example_tasklet);
}

module_init(my_init);
module_exit(my_exit);
```

---

9. 注意事项

- 不要在tasklet中调用可能睡眠的函数。
- tasklet适合短小、无需阻塞的延后处理任务。
- 不再推荐在新代码中大量使用tasklet，建议优先考虑workqueue。



## 工作队列处理机制：worker

Linux内核中的工作队列（workqueue）是一种用于实现内核异步处理的机制。它允许将需要延后或异步执行的任务放入队列，由专门的内核线程在合适的时机异步处理。工作队列机制在驱动开发、内核模块开发等场景中非常常用。

### 一、工作队列的核心概念

#### 1. 工作（work）

- 是需要被异步执行的“任务单元”。
- 在内核中用`struct work_struct`结构体来表示。

#### 2. 工作队列（workqueue）

- 是存放work的队列。
- 由内核线程（worker）负责取出队列中的work并执行。

#### 3. 内核线程

- 工作队列的任务是由worker线程（如`kworker`）调度执行的。

### 二、工作队列的主要数据结构

#### 1. `struct work_struct`

定义一个工作任务，其核心成员有：

```c
struct work_struct {
    atomic_long_t data; //传参数据
    struct list_head entry; //链表结构
    work_func_t func; // 回调函数，work被执行时会调用
    ...
};
```

#### 2. `struct workqueue_struct`

表示一个工作队列实例。管理worker线程及work任务，内部成员复杂，主要用于维护worker线程池和work队列。

#### 3. 操作方法：

```c
#include <linux/workqueue.h>
```

静态定义初始化：

```c
DECLARE_WORK(work_struct name,work_func_t f);
```

动态定义初始化：

```c
INIT_WORK(struct work_struct *work,work_func_t f);
```



工作调度：

```c
int schedule_work(struct work_struct *work);
```

取消工作：

```c
bool cancel_work_sync(struct work_struct *work);
```

### 三、工作队列的典型用法

#### 1. 声明和初始化work

- 静态初始化

  ```c
  void my_work_handler(struct work_struct *work);
  DECLARE_WORK(my_work, my_work_handler);
  ```

- 动态初始化

  ```c
  struct work_struct my_work;
  INIT_WORK(&my_work, my_work_handler);
  ```

#### 2. 提交work到工作队列

- 使用系统默认的全局工作队列：

  ```c
  schedule_work(&my_work);
  ```

- 向自定义工作队列提交：

  ```c
  struct workqueue_struct *my_wq;
  my_wq = create_workqueue("my_queue");
  queue_work(my_wq, &my_work);
  ```

#### 3. 销毁工作队列

- 对自定义工作队列：

  ```c
  destroy_workqueue(my_wq);
  ```

### 四、工作队列的执行流程

1. **提交任务**   用户调用`schedule_work`或`queue_work`把work放入工作队列。
2. **唤醒worker线程**   队列收到新work时，如果没有worker线程在运行，会唤醒一个worker线程。
3. **worker线程执行work**   worker线程从队列中取出work，调用其回调函数（func）。
4. **任务完成**   work执行结束后，worker线程继续处理下一个work，或者进入休眠。

### 五、工作队列的特点与优缺点

#### 优点

- **异步执行**：不阻塞当前上下文，适合不能在中断上下文中执行的任务。
- **线程上下文**：工作队列中的任务在进程上下文中运行，可以睡眠。
- **易用性**：内核提供了丰富的API和全局队列。

#### 缺点

- **有调度延迟**：任务不会立即执行，适合不需要立刻完成的工作。
- **内存占用**：每个work和worker线程都需消耗内存资源。
- **不能用于紧急任务**：如需立即执行的延迟任务，建议用tasklet或软中断。

### 六、与tasklet/软中断的区别

- **tasklet/软中断**：不能睡眠，执行时间短，优先级高。
- **工作队列**：可以睡眠，适合需要耗时或者需要等待的任务。

### 七、进阶特性

#### 1. 延迟工作（delayed_work）

- 可设置延迟时间后执行

  ```c
  struct delayed_work my_delayed_work;
  INIT_DELAYED_WORK(&my_delayed_work, my_work_handler);
  schedule_delayed_work(&my_delayed_work, msecs_to_jiffies(500)); // 500ms后执行
  ```

#### 2. 多处理器/并行队列

- 创建多worker的队列以提高并行度

  ```c
  create_workqueue("my_queue"); // 传统单线程
  alloc_workqueue("my_queue", WQ_UNBOUND, 0); // 多线程
  ```

### 八、实际应用场景

- 设备驱动中需在中断下半部处理的耗时任务
- 定时任务、后台数据处理
- 内核模块的异步处理需求

### 问题

#### 为什么 Linux 中会引入 workqueue 这个处理机制？

**目的和动机：**

- **内核中断处理上下文的限制**：在中断上下文（软中断、tasklet）中不允许睡眠，不能执行阻塞操作（如访问可能阻塞的 IO、获取可能阻塞的锁等）。
- **异步推后处理**：有些任务无需在中断中立即完成，可以推迟到内核线程上下文中执行（允许睡眠、阻塞），以提高中断响应速度和系统并发性。
- **统一异步任务框架**：workqueue 提供统一的、易用的异步任务调度和执行框架，简化了驱动和子系统的开发。

#### 数据结构是怎样？工作队列与工作之间什么关系？分别用于实现什么功能？

#### **主要数据结构：**

- **struct workqueue_struct 结构体 workqueue_struct**
  - 代表一个工作队列（workqueue），可以理解为一个管理一组内核线程的调度器。
  - 负责调度、分发、管理工作任务的执行。
- **struct work_struct 结构体 work_struct**
  - 代表一个具体的“工作”（work），即一个待执行的任务。
  - 本质是一个函数指针和其参数的包装体。

#### **关系与功能：**

- **工作队列（workqueue）**：负责调度和管理工作（work），将具体的任务挂到自己的队列中，由内核线程执行。
- **工作（work）**：实际要被执行的处理函数及其相关数据，被提交到工作队列后，由内核线程在合适的上下文中异步执行。

**关系类比：** 工作队列是“任务调度站”，工作是“等待处理的快递包裹”。

#### 工作原理是怎样？

**基本流程：**

1. **初始化工作（work）**：定义并初始化一个 `work_struct`，指定回调处理函数。
2. **提交工作到队列**：调用 API（如 `schedule_work()` 或 `queue_work()`）将工作提交到指定工作队列。
3. **内核线程处理**：工作队列内核线程会不断循环检查队列，有新工作时调用其处理函数执行。
4. **处理完成**：回调函数运行完毕后，工作即完成。

**注意：**

- **默认工作队列**：Linux 内核维护一个全局的默认工作队列（system_wq）。
- **自定义工作队列**：可根据需要创建专用的工作队列，支持更细致的资源管理和 CPU 亲和性。
- **延时工作**：可以提交定时/延时执行的工作（delayed_work）。

## 线程KThread

### 1. 内核线程的基本概念

#### 1.1 什么是内核线程

- 内核线程（Kernel Thread）是在内核空间运行的特殊进程，用于执行内核中的特定任务（如内存回收、定时器、IO调度等）。
- 与普通进程不同，内核线程通常不需要用户空间的地址空间，也不与终端、文件等用户资源绑定，且通常不会退出到用户态。

#### 1.2 内核线程与用户线程的区别

- 用户线程运行在用户空间，由用户进程创建和管理，切换时需要上下文切换到内核。
- 内核线程运行在内核空间，属于系统进程，直接由内核调度和管理，切换开销更小。

### 2. 内核线程的创建与管理

#### 2.1 内核线程的创建

- 早期Linux使用`kernel_thread()`系统调用创建内核线程，但自2.6以后推荐使用`kthread`接口。
- 典型创建流程：
  - 使用`kthread_create(function, data, name, ...)`创建线程，但线程不会立即运行。
  - 需要调用`wake_up_process(task_struct)`或`kthread_run()`来激活线程。

**示例代码：**

```c
struct task_struct *task;
task = kthread_create(thread_function, data, "my_kthread");
wake_up_process(task);
// 或者更常用的写法：
task = kthread_run(thread_function, data, "my_kthread");
```

#### 2.2 内核线程结构

- 内核线程其实也是一种进程，结构体是`task_struct`，区别主要体现在`mm`成员为NULL（不需要用户空间内存）。

## 三种处理机制的对比

#### 1. **小任务（Tasklet）**

##### **概念**

- Tasklet是Linux内核中一种轻量级的软中断下半部处理机制。
- 主要用于在中断上下文下延迟执行的简单任务。

##### **特点**

- 运行在中断上下文中，**不能睡眠**。
- 只适合执行短小、快速的任务。
- 不能进行可能阻塞的操作（如访问可能睡眠的资源、调用可能阻塞的API）。
- 优先级比工作队列高，但不如硬中断。

##### **使用场景**

- 适用于对时间要求比较高的、简单的中断后处理任务。

------

#### 2. **工作队列（Workqueue）**

##### **概念**

- 工作队列是内核中一种将任务交由内核线程在进程上下文中执行的机制。
- 允许任务延后执行，且**可以睡眠和阻塞**。

##### **特点**

- 运行在进程上下文（内核线程）中，**可以睡眠**。
- 适合执行较复杂、可能阻塞的下半部任务（如需要等待资源、进行I/O等）。
- 优先级比tasklet低，执行时机更灵活。
- 可以通过自定义工作队列实现并发或串行处理。

##### **使用场景**

- 适用于需要延迟处理、可能会阻塞的任务，如复杂的设备驱动下半部、文件系统操作等。

------

#### 3. **线程（Kernel Thread / kthread）**

##### **概念**

- 线程（通常指内核线程kthread）是内核级别的轻量级进程。
- 可以像普通进程一样拥有独立的调度和资源，可以执行任意内核任务。

##### **特点**

- 运行在进程上下文，可以睡眠或阻塞。
- 适合处理**长期的、复杂的、需要持续运行或周期性工作**的任务。
- 线程可以被唤醒、挂起、终止、与其他线程进行同步等。
- 资源消耗相对tasklet和workqueue更大，但功能最灵活。

##### **使用场景**

- 适合需要持续运作、周期性任务、调度复杂、需要大量状态维护的内核服务或驱动。

------

**对比总结表**

| 特性         | 小任务 (tasklet) | 工作队列 (workqueue) | 线程 (kthread)     |
| ------------ | ---------------- | -------------------- | ------------------ |
| 上下文       | 中断上下文       | 进程上下文           | 进程上下文         |
| 是否可睡眠   | 否               | 是                   | 是                 |
| 是否可阻塞   | 否               | 是                   | 是                 |
| 适合任务类型 | 短小、简单       | 较复杂、可阻塞       | 长期、复杂、循环   |
| 优先级       | 高于workqueue    | 低于tasklet          | 可调度             |
| 使用场景     | 短时中断下半部   | 复杂下半部、延迟任务 | 后台服务、守护线程 |

------

**实际举例**

- **tasklet**：网卡驱动包接收后的快速处理、软中断处理。
- **workqueue**：USB驱动中数据传输后的复杂处理、定时任务。
- **线程（kthread）**：定时同步守护、块设备IO管理、内存回收等长期任务。

------

#### **总结**

- **tasklet**：最轻量、最快速，不能阻塞，适合极短任务。
- **workqueue**：灵活、可阻塞，适合复杂但不需要一直运行的任务。
- **kthread**：最灵活、功能最强、资源消耗大，适合长期/复杂/周期性任务。

## 时间管理

### 关于时间管理

1.  处理器可以循环执行时间固定的指令
2. 定时器电路通过计数器来表达时间
3. 操作系统通过软硬件结合来实现
4. 应用程序主要是通过调用系统提供的封装好的时间接口来实现

### Linux时间管理

#### HZ与jiffies

HZ，表示硬件周期性时间间隔，比如ARM：HZ =256/s

jiffies，表示系统自启动后发性中断次数，是一个定义在Linux全局变量，启动时jiffies=0，之后根据HZ自动递增1

若用户想要计算自启动后到现在的系统时间t = jiffies * 1/HZ

计算从现在到未来5s后：

jiffies = jiffies + 5 * HZ

```c
//设计一个以秒为单位时间延时函数：
void delay(unsigned long sec)
{
	tcnt = jiffies + sec * HZ;
	while(jiffies < tcnt);
}
```

### 时间接口

#### 1. 时间延时

```c
#include <linux/delay.h>

//等待延时函数
void ndelay(unsigned long nsec);	//ns
void udelay(unsigned long usec);	//us
void mdelay(unsigned long msec);	//ms

//休眠延时函数
void ssleep(unsigned int seconds);	//s
void msleep(unsigned int msecs));	//ms
```

##### 等待延时函数（Busy-wait Delay，轮询延时）

- **实现原理**：通过一个循环不断检查条件或计数来“等待”一段时间。
- **CPU占用**：在延时期间，CPU一直在运行循环体（不断执行指令），不会让出控制权。
- **常见场景**：嵌入式系统、单片机底层程序、对延时精度要求高但资源有限的地方。
- 优点：
  - 实现简单。
  - 延时精度较高，可以精确到CPU周期级别。
- 缺点：
  - 资源浪费，CPU无法做其他事情，效率低下。
  - 不适合多任务/多线程环境。

------

##### 2. 休眠延时函数（Sleep Delay，阻塞延时）

- **实现原理**：调用操作系统提供的休眠接口，让当前线程/进程“睡眠”，操作系统调度其它任务。
- **CPU占用**：延时时间内，CPU可以切换去执行别的任务，当前线程/进程被挂起或阻塞。
- **常见场景**：操作系统应用开发、需要高效利用CPU资源、多线程/多进程环境。
- 优点：
  - CPU资源充分利用，不会因为等待而浪费CPU时间片。
  - 适合多任务系统。
- 缺点：
  - 精度受操作系统调度影响，通常达不到毫秒以下的高精度。

------

##### 3. 总结对比

| 特性             | 等待延时函数（忙等） | 休眠延时函数（sleep） |
| ---------------- | -------------------- | --------------------- |
| CPU占用          | 高                   | 低                    |
| 精度             | 高                   | 受调度影响            |
| 是否阻塞其他任务 | 阻塞                 | 不阻塞                |
| 场景             | 单片机、嵌入式       | 系统级应用，多任务    |

------

##### 4. 适用场景建议

- **需要极高精度、无操作系统环境**——用等待延时（忙等）。
- **普通延时、需高效利用CPU、多任务场景**——用休眠延时（sleep类函数）。

#### 2.时间定时

在Linux内核中，实现了软件定时器功能

```c
#include <linux/timer.h>
```

##### 重要结构

时间描述结构体

```c
struct tvec_base {
	spinlock_t lock;
	struct timer_list *running_timer;
	unsigned long timer_jiffies;
	unsigned long next_timer;
	struct tvec_root tv1;
	struct tvec tv2;
	struct tvec tv3;
	struct tvec tv4;
	struct tvec tv5;
} ____cacheline_aligned;
```

定时器描述结构体

```c
struct timer_list {
	/*
	 * All fields that change during normal runtime grouped to the
	 * same cacheline
	 */
	struct list_head entry;		//链表结构
	unsigned long expires;		//要设定的未来jiffies值
	struct tvec_base *base;		//时间

	void (*function)(unsigned long);	//定时触发后要执行的处理函数
	unsigned long data;					//处理函数的传参变量

	int slack;
}
```

##### 常用的操作方法

1. 

   定义并初始化软件定时器

   ```c
   DEFINE_TIMER(_name,_function,_expires,_data);
   ```

   初始化软件定时器；

   ```c
   void init_timer(struct timer_list *timer);
   ```

2. 

   启用软件定时器

   ```c
   void add_timer(struct timer_list *timer);
   ```

   停止软件定时器

   ```c
   int del_timer(struct timer_list *timer);
   ```

## 休眠唤醒机制

### 一、什么是休眠唤醒机制？

在Linux内核开发中，**休眠（sleep）**和**唤醒（wake up）**机制用于线程/进程间的同步与互斥。当某个任务（进程/线程）因为等待某种事件（如I/O、资源等）无法继续执行时，它会进入休眠状态。等事件到来时，再将其唤醒，恢复执行。

---

### 二、休眠和唤醒的原理

#### 1. 休眠（Sleep）
- 进程/线程主动让出CPU，自身进入**不可运行（TASK_UNINTERRUPTIBLE/TASK_INTERRUPTIBLE）**的状态。
- 进入休眠后，调度器会把CPU资源分配给别的可运行进程。

#### 2. 唤醒（Wake up）
- 当等待的事件发生时，通过唤醒机制把休眠的进程/线程状态恢复为**可运行（TASK_RUNNING）**，等待调度器重新调度执行。

#### 3. 重要结构体

内核链表结构体

```c
struct list_head {
	struct list_head *next, *prev;
};
```

等待队列头结构体

```c
struct __wait_queue_head {
	spinlock_t lock;
	struct list_head task_list;
};
```

等待队列结构

```c
typedef int (*wait_queue_func_t)(wait_queue_t *wait, unsigned mode, int flags, void *key);
struct __wait_queue {
	unsigned int flags;
#define WQ_FLAG_EXCLUSIVE	0x01
	void *private;
	wait_queue_func_t func;
	struct list_head task_list;
};
```

---

### 三、内核实现机制

#### 1. 等待队列（Wait Queue）

Linux内核通过**等待队列**来管理休眠和唤醒的任务。

##### 1.1 数据结构
- `struct wait_queue_head`：等待队列头
- `struct wait_queue_entry`（或旧版本`wait_queue_t`）：等待队列项，通常由进程/线程填充

##### 1.2 常用接口

##### 初始化等待队列
```c
DECLARE_WAIT_QUEUE_HEAD(my_queue);         // 静态初始化
wait_queue_head_t my_queue;
init_waitqueue_head(&my_queue);            // 动态初始化
```

---

#### 2. 进入休眠（常见函数）

##### 2.1 休眠函数
- `wait_event(queue, condition)`：如果condition为假则当前进程休眠，直到condition为真。
- `wait_event_interruptible(queue, condition)`：类似，但休眠过程可以被信号中断（如Ctrl+C）。
- `wait_event_timeout(queue, condition, timeout)`：带超时的休眠。

##### 源码示例

```c
wait_event(queue, data_ready == 1);
```
> 若`data_ready`为0，则当前进程加入`queue`队列并休眠，等其他任务唤醒。

---

#### 3. 唤醒机制

##### 3.1 唤醒函数
- `wake_up(&queue)`：唤醒在该队列上等待的所有进程。
- `wake_up_interruptible(&queue)`：唤醒可中断休眠（wait_event_interruptible）进程。

##### 源码示例

```c
data_ready = 1;
wake_up(&queue);
```
> 这样就会唤醒所有因`wait_event`而休眠的进程。

---

### 四、工作流程举例

1. **生产者-消费者模型**
   - 消费者进程获取不到数据时，调用`wait_event`休眠在等待队列上。
   - 生产者进程生产数据后，设置数据就绪标志，然后调用`wake_up`唤醒消费者。

2. **阻塞I/O模型**
   - 读操作发现无数据时，休眠等待。
   - 数据到来后，设备驱动通过`wake_up`唤醒读进程。

---

### 五、常见休眠与唤醒相关接口

| 接口名                   | 作用               |
| ------------------------ | ------------------ |
| DECLARE_WAIT_QUEUE_HEAD  | 静态定义等待队列   |
| init_waitqueue_head      | 动态初始化等待队列 |
| wait_event               | 事件等待，休眠     |
| wait_event_interruptible | 可被信号中断的休眠 |
| wait_event_timeout       | 超时等待           |
| wake_up                  | 唤醒等待队列       |
| wake_up_interruptible    | 唤醒可中断等待队列 |

---

### 六、注意事项

1. **休眠前必须有可唤醒条件**，否则容易死锁。
2. 使用`wait_event_interruptible`时要判断返回值（是否被信号中断）。
3. 不能在**中断上下文**中休眠（否则会导致内核崩溃），工作队列、内核线程中是可以的。
4. 休眠和唤醒配合等待队列使用，避免竞态条件。

---

### 七、经典代码示例

```c
#include <linux/wait.h>

DECLARE_WAIT_QUEUE_HEAD(my_queue);
int data_ready = 0;

// 线程A，等待数据
wait_event(my_queue, data_ready == 1); // 休眠直到data_ready变为1

// 线程B，生产数据
data_ready = 1;
wake_up(&my_queue); // 唤醒线程A
```

---

### 八、进阶：原理图解

```text
休眠前
  +-----------------+        +--------------------+
  | 进程A           |        | 等待队列           |
  +-----------------+        +--------------------+

休眠调用 wait_event
  +-----------------+        +--------------------+
  | 进程A（休眠）   |<------ | my_queue           |
  +-----------------+        +--------------------+

事件发生唤醒
  +-----------------+        +--------------------+
  | 进程A（唤醒）   | -----> | my_queue           |
  +-----------------+        +--------------------+
```

---

### 九、总结

- **休眠**：让出CPU，当前进程“睡觉”，避免忙等浪费CPU资源。
- **唤醒**：事件发生后，把进程“叫醒”，恢复执行。
- **等待队列**：内核用来管理所有休眠进程，配合休眠/唤醒机制实现高效同步。



## 初级设备驱动

### 相关概念

#### 设备文件

设备文件是在顶层应用空间中（/dev/），用于表示底层设备在应用空间中的一个符号（链接）。应用程序就是通过打开这个文件，找到系统层中的驱动，再调用驱动来驱动设备

设备文件 = 字符设备文件（c） +  块设备文件（b）

创建设备文件：

```
mknod 设备文件名称 设备类型 主设备号 次设备号
```

如：

mknod /dev/myled c 88 0

在linux中，使用“设备驱动管理子系统”来维护与管理驱动，每个驱动都会有个编号，这就是设备号。

```
设备号 = 主设备号(major) + 次设备号(minor)
```

#### 设备类型

在linux中，所有设备都是先进行分类，再进行处理

##### 字符设备类 char

跟存储无关，跟网络无关的其他设备，usb、uart...

##### 块设备类 block

跟存储相关的设备，sd、mmc、u盘

##### 网络设备类 socket

跟网络通信有关的设备，DM9000、cs8900、wifi

### 初级驱动框架初探

![image-20250710215214289](F:\bound\aic learn\笔记\assets\image-20250710215214289-1752155544086-1.png)

这幅图描绘了 Linux 初级设备驱动原理的架构与流程。最上层是用户程序，应用工程师在用户应用层，通过系统调用（如 open 等）与内核进行交互，实现对设备的操作。内核中包含系统调用接口，如 sys_open、sys_read、sys_write 等，它们是用户程序与内核驱动程序之间的桥梁。

在内核层，有设备驱动管理子系统，负责管理驱动。其中又可以分为设备驱动管理子系统和网络管理子系统。驱动是设备驱动程序的核心部分，它实现对特定硬件设备的具体操作，比如读取设备寄存器、控制设备状态等。设备则是具体硬件的抽象表示。

设备驱动程序会向内核注册自己，表明可以管理哪些设备。当用户程序发起对设备的操作请求时，系统调用接口会将请求传递给设备模型驱动管理子系统，后者根据设备类型和总线关系，找到对应的驱动程序来处理请求。驱动程序进而对硬件设备进行实际的操作，完成数据的读写、设备状态的控制等功能，从而实现用户程序对硬件设备的访问和控制，使得 Linux 系统能够与各种硬件设备协同工作，发挥相应功能。

驱动程序由驱动工程师开发，如led.ko会将自己注册到驱动管理子系统的字符设备下面，其中封装了一些对硬件电路（led灯）操作的函数接口。而应用开发工程师则通过系统调用控制底层硬件。应用程序通过open等系统调用接口找到对应设备文件，glibc封装为系统调用后，内核sys_open()系统调用处理，给到虚拟文件系统解析路径，再给到设备管理子系统解析判断出是设备文件读取其设备号，内核根据主设备号查找字符设备驱动表，就找到led.ko注册的file_operations结构，在其中的file_operations结构体中找到封装的接口函数，执行并返回，然后内核再创建文件描述符返回给用户空间，用户应用程序就得到了文件描述符fd。

### 利用初级驱动框架开发led驱动

封装于<linux/fs.h>中

#### 常用数据结构

##### 1. cdev

```c
struct cdev {
    struct kobject kobj;                    // 内核对象，用于引用计数和sysfs文件系统集成
    struct module *owner;                   // 指向拥有此字符设备的模块，用于模块引用计数
    const struct file_operations *ops;     // 指向文件操作函数表，定义设备的操作接口
    struct list_head list;                  // 链表节点，用于将设备链接到全局字符设备链表
    dev_t dev;                             // 设备号，包含主设备号和次设备号
    unsigned int count;                     // 此cdev结构管理的连续次设备号数量
};
```

##### 2. file

```c
struct file {
    struct path f_path;                         // 文件路径信息，包含dentry和vfsmount
    const struct file_operations *f_op;        // 指向文件操作函数表，定义文件的具体操作
    atomic_long_t f_count;                      // 文件引用计数，记录有多少进程打开了此文件
    unsigned int f_flags;                       // 文件标志位，如O_RDONLY、O_WRONLY、O_APPEND等
    fmode_t f_mode;                            // 文件访问模式，如FMODE_READ、FMODE_WRITE等
    loff_t f_pos;                              // 文件当前读写位置偏移量
    void *private_data;                        // 驱动或文件系统私有数据指针
};
```

##### 3. file_operations

```c
struct file_operations {
    ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);     
    // 读操作函数指针
    ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *); 
    // 写操作函数指针
    int (*open) (struct inode *, struct file *);                          
    // 打开文件操作函数指针
    int (*flush) (struct file *, fl_owner_t id);                        
    // 刷新缓冲区函数指针
    int (*release) (struct inode *, struct file *);                      
    // 释放文件操作函数指针
    
    //...还有很多其他的成员
};
```

#### 静态驱动注册与注销

```c
#include <linux/fs.h>

int register_chrdev(unsigned int major,	//注册成功后分配到的设备号
                                  const char *name,		//设备名
				  const struct file_operations *fops	//指向驱动的方法
                                 )
    
void unregister_chrdev(unsigned int major, const char *name)
```

需要自己指定设备号

#### 动态驱动注册与注销

```c
// 动态分配设备号
int alloc_chrdev_region(dev_t *dev, unsigned baseminor, unsigned count, const char *name);

// 释放设备号
void unregister_chrdev_region(dev_t from, unsigned count);

// 初始化cdev结构体
void cdev_init(struct cdev *cdev, const struct file_operations *fops);

// 添加字符设备到系统
int cdev_add(struct cdev *p, dev_t dev, unsigned count);

// 从系统中删除字符设备
void cdev_del(struct cdev *p);
```

自动分配设备号，通过dev_t指针返回

#### 创建设备类和创建设备文件

##### 一、为什么要有“类”和“自动节点”

1. 早期做法
   用 `register_chrdev()` 或 `alloc_chrdev_region()` 只在内核里登记了一个“主设备号 + file_operations”。
   用户空间必须手动

   ```
   mknod /dev/xxx c 250 0
   ```

   主/次设备号一旦变化就要重新 mknod，非常痛苦。

2. 现代做法（udev/mdev）

   - 内核把“设备信息”通过 netlink 广播到用户空间。
   - udev 收到后根据规则在 `/dev` 下创建设备节点。
   - 为了让 udev 知道“这个字符设备应该叫啥、权限是多少、放在哪个子目录”，内核必须：
     1. 先建一个“逻辑分组”——**class**（`/sys/class/xxx/`）。
     2. 再在 class 下挂一个“设备”——**device**（`/sys/devices/virtual/xxx/yyy/`）。
        这样 udev 才能找到 `/sys/class/xxx/yyy/dev`，读取主次设备号，再 mknod。

------

##### 二、代码骨架

下面给出最小可工作的模板，随后逐行拆解。

```c
#include <linux/device.h>   /* class_create / device_create */

static dev_t devid;         /* 保存主次设备号 */
static struct cdev my_cdev;
static struct class  *my_class;

static int __init my_init(void)
{
    int ret;

    /* 1. 申请设备号 */
    ret = alloc_chrdev_region(&devid, 0, 2, "myled");  /* 次设备号从0开始，共2个 */
    if (ret)
        return ret;

    /* 2. 初始化并添加 cdev */
    cdev_init(&my_cdev, &my_fops);
    ret = cdev_add(&my_cdev, devid, 2);
    if (ret)
        goto err_cdev;

    /* 3. 创建 class（/sys/class/myled） */
    my_class = class_create(THIS_MODULE, "myled");
    if (IS_ERR(my_class)) {
        ret = PTR_ERR(my_class);
        goto err_class;
    }

    /* 4. 在 class 下创建设备（/dev/myled0, /dev/myled1） */
    device_create(my_class, NULL, MKDEV(MAJOR(devid), 0), NULL, "myled0");
    device_create(my_class, NULL, MKDEV(MAJOR(devid), 1), NULL, "myled1");

    return 0;

err_class:
    cdev_del(&my_cdev);
err_cdev:
    unregister_chrdev_region(devid, 2);
    return ret;
}

static void __exit my_exit(void)
{
    /* 反向销毁：device → class → cdev → region */
    device_destroy(my_class, MKDEV(MAJOR(devid), 0));
    device_destroy(my_class, MKDEV(MAJOR(devid), 1));
    class_destroy(my_class);
    cdev_del(&my_cdev);
    unregister_chrdev_region(devid, 2);
}
module_init(my_init);
module_exit(my_exit);
```

------

##### 三、逐行拆解

1. `struct class *class_create(owner, name)`

   - 作用：在 `/sys/class/` 目录下新建一个名为 `name` 的子目录。
   - 参数
     - `owner`: 一般传 `THIS_MODULE`，用于引用计数。
     - `name`: 目录名，尽量与驱动名一致，如 `"myled"`。
   - 返回值
     - 成功：指向 `struct class` 的指针。
     - 失败：`IS_ERR()` 为真，错误码用 `PTR_ERR()` 取出。
   - 注意
     如果 name 与已有 class 同名，返回 `-EEXIST`。
     卸载时必须 `class_destroy()`，否则 `/sys/class/xxx` 残留。

2. `struct device *device_create(class, parent, devt, drvdata, fmt, ...)`

   - 作用：在 class 目录下创建一个逻辑设备，同时通过 uevent 触发 udev 创建设备节点 `/dev/xxx`。

   - 参数（重点）

     - `class`: 刚才 `class_create()` 返回的指针。
     - `parent`: 物理父设备，可为 NULL（表示 virtual device）。
     - `devt`: `MKDEV(major, minor)`，告诉 udev 主次设备号。
     - `drvdata`: 私有数据指针，会保存在 `device->driver_data`，可为 NULL。
     - `fmt`: 设备节点名字，支持 printf 格式化。

   - 返回值

     - 成功：指向 `struct device` 的指针。
     - 失败：`IS_ERR()` 为真。

   - 常见用法

     ```
     device_create(my_class, NULL, MKDEV(major, 0), NULL, "myled%d", 0);
     ```

     这样 udev 会生成 `/dev/myled0`（权限 600，root:root，可通过 udev 规则修改）。

3. 清理镜像

   - `void device_destroy(struct class *cls, dev_t devt)`
     删除 `/sys/class/xxx/yyy`，并触发 udev 删除 `/dev/yyy`。
     注意第二个参数 **必须是当初创建设备时用的 devt**，否则找不到。
   - `void class_destroy(struct class *cls)`
     删除 `/sys/class/xxx` 目录。



#### 源码阅读

##### 动态分配设备号

```c
int alloc_chrdev_region(dev_t *dev, unsigned baseminor, unsigned count,
			const char *name)
{
	struct char_device_struct *cd;
	cd = __register_chrdev_region(0, baseminor, count, name);
	if (IS_ERR(cd))
		return PTR_ERR(cd);
	*dev = MKDEV(cd->major, cd->baseminor);
	return 0;
}
```

首先定义一个char_device_struct结构体，这个结构体中包含了设备号和设备数量

```c
struct cdev {
    struct kobject kobj;                // kobject基础对象
    struct module *owner;               // 所属模块
    const struct file_operations *ops;  // 文件操作
    struct list_head list;              // 链表节点
    dev_t dev;                          // 设备号
    unsigned int count;                 // 设备数量
};
```

在__register_chrdev_region函数中，先申请了一块char_device_struct大小的空间，并判断传入的主设备号是否为0，为0则从chrdevs数组中从后往前检查看是否存在空的主设备号，存在则进行分配。同时也将次设备号等进行了分配。

而MKDEV则是将主设备号进行了左移，与次设备号拼接在了一起。

而cdev_init则是将cdev结构体中的file_operations指向定义好的结构体。

```c
void cdev_init(struct cdev *cdev, const struct file_operations *fops)
{
	memset(cdev, 0, sizeof *cdev);
	INIT_LIST_HEAD(&cdev->list);
	kobject_init(&cdev->kobj, &ktype_cdev_default);
	cdev->ops = fops;
}
```

而最后则通过cdev_add将设备号给到cdev结构体。

```c
int cdev_add(struct cdev *p, dev_t dev, unsigned count)
{
	p->dev = dev;
	p->count = count;
	return kobj_map(cdev_map, dev, count, NULL, exact_match, exact_lock, p);
}
```



## 板级文件与设备树

------

### 1. 板级文件（BSP/board.c）

- **概念**：
  板级文件是早期Linux内核支持具体硬件平台时的主要方式，一般是C语言源码（例如 board.c、mach-xx.c、bsp.c 等）。它直接在内核源码中用代码描述硬件的各项信息和初始化方法。

- **作用**：

  - 初始化CPU、内存、外设等
  - 指定和注册各种外设的资源信息（如地址、中断、GPIO编号等）
  - 进行板卡专有的初始化逻辑

- **特点**：

  - **强耦合**：代码和具体硬件板卡紧密绑定，移植性差。

  - 修改硬件信息需要改代码并重新编译内核。

  - 常见于老的ARM、MIPS等嵌入式Linux内核（3.x及更早）。

  - 例子（伪代码）：

    C

    ```
    static struct platform_device my_uart = {
        .name = "my-uart",
        .id = 0,
        .resource = ..., // 硬件资源（地址/中断）
    };
    ```

------

### 2. 设备树（Device Tree，*.dts）

- **概念**：
  设备树是一种**硬件描述语言**，用数据结构（而不是C代码）来描述硬件信息。Linux在启动时解析设备树，自动生成和配置硬件资源。

- **作用**：

  - 独立于内核代码描述硬件结构和资源
  - 动态传递给内核（通常通过bootloader）
  - 支持硬件平台的灵活移植和定制

- **特点**：

  - **解耦**：硬件信息与内核代码分离，便于维护和移植。

  - 更改硬件资源不需要修改内核代码，只需改dts文件。

  - 方便支持同一内核适配多个硬件平台。

  - 例子（伪代码）：

    dts

    ```
    uart0: serial@101f1000 {
        compatible = "arm,pl011";
        reg = <0x101f1000 0x1000>;
        interrupts = <5>;
    };
    ```

------

### 3. 主要区别总结

| 比较点         | 板级文件（BSP）      | 设备树（Device Tree）            |
| -------------- | -------------------- | -------------------------------- |
| 表达方式       | C代码实现            | 数据结构（*.dts/*.dtsi文件）     |
| 位置           | 内核源码的一部分     | 独立于内核源码                   |
| 修改方式       | 需重新编译内核       | 修改dts文件即可                  |
| 移植性         | 差，每块板需专有代码 | 好，可支持多平台                 |
| 适用内核版本   | 老版本（3.x及更早）  | 新内核（主流ARM/MIPS/PowerPC等） |
| 驱动与硬件绑定 | 强耦合               | 解耦                             |
| 维护成本       | 高                   | 低                               |

------

### 4. 联系

- 设备树大大减少了板级文件的代码量，但部分特殊初始化代码（如电源管理、复位逻辑等）在现代Linux中，**仍有少量板级代码配合设备树使用**。
- 设备树的出现是为了解决板级文件膨胀和维护困难的问题。

------

### 5. 结论

- **板级文件**：通过C代码实现，内核和硬件紧密绑定，移植性差，维护繁琐，适合早期或简单硬件平台。
- **设备树**：通过数据结构描述硬件，极大提高了移植性和灵活性，是现代主流Linux平台的推荐做法。

## 总结

### 1. 模块管理函数

### 模块初始化和退出

```c
#include <linux/module.h>
#include <linux/init.h>

module_init(init_function);     // 模块初始化
module_exit(exit_function);     // 模块退出
MODULE_LICENSE("GPL");          // 模块许可证
MODULE_AUTHOR("author");        // 模块作者
MODULE_DESCRIPTION("desc");     // 模块描述
MODULE_VERSION("1.0");          // 模块版本
```

### 2. 内存管理函数

### 内存分配

```c
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <asm/io.h>

// 内核内存分配
void *kmalloc(size_t size, gfp_t flags);
void kfree(const void *ptr);

// 连续页面分配
void *__get_free_pages(gfp_t flags, unsigned int order);
void free_pages(unsigned long addr, unsigned int order);

// 虚拟内存分配
void *vmalloc(unsigned long size);
void vfree(const void *addr);

// I/O内存映射
void __iomem *ioremap(phys_addr_t phys_addr, size_t size);
void iounmap(volatile void __iomem *addr);
```

### 内存拷贝

```c
#include <asm/uaccess.h>

// 内核空间与用户空间数据传输
unsigned long copy_to_user(void __user *to, const void *from, unsigned long n);
unsigned long copy_from_user(void *to, const void __user *from, unsigned long n);

// 内核空间内存拷贝
void *memcpy(void *dest, const void *src, size_t n);
void *memset(void *s, int c, size_t n);
```

### 3. 字符设备驱动函数

### 设备注册

```c
#include <linux/fs.h>
#include <linux/cdev.h>

// 字符设备注册
int register_chrdev(unsigned int major, const char *name, 
                    const struct file_operations *fops);
void unregister_chrdev(unsigned int major, const char *name);

// 现代字符设备接口
int alloc_chrdev_region(dev_t *dev, unsigned baseminor, 
                        unsigned count, const char *name);
void unregister_chrdev_region(dev_t from, unsigned count);

// 字符设备结构
struct cdev *cdev_alloc(void);
void cdev_init(struct cdev *cdev, const struct file_operations *fops);
int cdev_add(struct cdev *p, dev_t dev, unsigned count);
void cdev_del(struct cdev *p);
```

### 4. 设备类和设备节点

### 设备类管理

```c
#include <linux/device.h>

// 创建设备类
struct class *class_create(struct module *owner, const char *name);
void class_destroy(struct class *cls);

// 创建设备节点
struct device *device_create(struct class *cls, struct device *parent,
                            dev_t devt, void *drvdata, const char *fmt, ...);
void device_destroy(struct class *cls, dev_t devt);
```

### 5. 中断处理函数

### 中断注册和处理

```c
#include <linux/interrupt.h>

// 中断注册
int request_irq(unsigned int irq, irq_handler_t handler, 
                unsigned long flags, const char *name, void *dev);
void free_irq(unsigned int irq, void *dev_id);

// 中断使能/禁用
void disable_irq(unsigned int irq);
void enable_irq(unsigned int irq);
void local_irq_disable(void);
void local_irq_enable(void);
```

### 6. 同步和互斥函数

### 信号量

```c
#include <linux/semaphore.h>

void sema_init(struct semaphore *sem, int val);
void down(struct semaphore *sem);
int down_interruptible(struct semaphore *sem);
int down_trylock(struct semaphore *sem);
void up(struct semaphore *sem);
```

### 互斥锁

```c
#include <linux/mutex.h>

void mutex_init(struct mutex *lock);
void mutex_lock(struct mutex *lock);
int mutex_lock_interruptible(struct mutex *lock);
int mutex_trylock(struct mutex *lock);
void mutex_unlock(struct mutex *lock);
```

### 自旋锁

```c
#include <linux/spinlock.h>

void spin_lock_init(spinlock_t *lock);
void spin_lock(spinlock_t *lock);
void spin_unlock(spinlock_t *lock);
void spin_lock_irqsave(spinlock_t *lock, unsigned long flags);
void spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags);
```

### 7. 等待队列

### 等待队列操作

```c
#include <linux/wait.h>

// 初始化等待队列
init_waitqueue_head(wait_queue_head_t *q);

// 等待事件
wait_event(wq, condition);
wait_event_interruptible(wq, condition);
wait_event_timeout(wq, condition, timeout);

// 唤醒等待队列
wake_up(wait_queue_head_t *q);
wake_up_interruptible(wait_queue_head_t *q);
```

### 8. 定时器函数

### 内核定时器

```c
#include <linux/timer.h>

// 定时器操作
void timer_setup(struct timer_list *timer, void (*callback)(struct timer_list *), 
                 unsigned int flags);
int mod_timer(struct timer_list *timer, unsigned long expires);
int del_timer(struct timer_list *timer);
int del_timer_sync(struct timer_list *timer);
```

### 延时函数

```c
#include <linux/delay.h>

void udelay(unsigned long usecs);    // 微秒延时
void mdelay(unsigned long msecs);    // 毫秒延时
void msleep(unsigned int msecs);     // 毫秒睡眠
void ssleep(unsigned int seconds);   // 秒睡眠
```

### 9. 工作队列

### 工作队列操作

```c
#include <linux/workqueue.h>

// 创建工作队列
struct workqueue_struct *create_workqueue(const char *name);
void destroy_workqueue(struct workqueue_struct *wq);

// 工作项操作
INIT_WORK(struct work_struct *work, work_func_t func);
bool queue_work(struct workqueue_struct *wq, struct work_struct *work);
bool schedule_work(struct work_struct *work);
```

### 10. 调试和日志函数

### 内核日志

```c
#include <linux/printk.h>

printk(KERN_EMERG "Emergency message\n");
printk(KERN_ALERT "Alert message\n");
printk(KERN_CRIT "Critical message\n");
printk(KERN_ERR "Error message\n");
printk(KERN_WARNING "Warning message\n");
printk(KERN_NOTICE "Notice message\n");
printk(KERN_INFO "Info message\n");
printk(KERN_DEBUG "Debug message\n");
```

### 设备相关日志

```c
#include <linux/device.h>

dev_err(dev, "Error message\n");
dev_warn(dev, "Warning message\n");
dev_info(dev, "Info message\n");
dev_dbg(dev, "Debug message\n");
```

### 11. GPIO操作函数

### GPIO基本操作

```c
#include <linux/gpio.h>

// GPIO申请和释放
int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);

// GPIO方向设置
int gpio_direction_input(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);

// GPIO值操作
int gpio_get_value(unsigned gpio);
void gpio_set_value(unsigned gpio, int value);
```

### 12. 平台设备驱动

### 平台驱动注册

```c
#include <linux/platform_device.h>

// 平台驱动注册
int platform_driver_register(struct platform_driver *drv);
void platform_driver_unregister(struct platform_driver *drv);

// 资源获取
struct resource *platform_get_resource(struct platform_device *dev,
                                      unsigned int type, unsigned int num);
int platform_get_irq(struct platform_device *dev, unsigned int num);
```

### 13. 常用宏和辅助函数

### 错误处理

```c
#include <linux/err.h>

bool IS_ERR(const void *ptr);
long PTR_ERR(const void *ptr);
void *ERR_PTR(long error);
```

### 容器操作

```c
#include <linux/kernel.h>

// 获取包含结构
container_of(ptr, type, member);

// 数组大小
ARRAY_SIZE(arr);

// 最大最小值
min(x, y);
max(x, y);
```
