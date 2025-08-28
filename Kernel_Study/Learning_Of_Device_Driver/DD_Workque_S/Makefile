obj-m += workque.o

KERNEL_DIR = /opt/kernel-build/kernel-3.4.39/
GCC_TOOL = /usr/local/arm/arm-eabi-4.8/bin/arm-eabi-

all:
	make modules -C $(KERNEL_DIR) M=`pwd` CROSS_COMPILE=$(GCC_TOOL)

clean:
	make modules clean -C $(KERNEL_DIR) M=`pwd` CROSS_COMPILE=$(GCC_TOOL)
