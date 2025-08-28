
KERNEL_DIR = /opt/kernel-build/kernel-3.4.39/

obj-m += led.o

all:
	make modules M=`pwd` -C $(KERNEL_DIR) CROSS_COMPILE=/usr/local/arm/arm-eabi-4.8/bin/arm-eabi-
	arm-linux-gcc app.c -o app
clean:
	make modules clean M=`pwd` -C  $(KERNEL_DIR) CROSS_COMPILE=/usr/local/arm/arm-eabi-4.8/bin/arm-eabi-
	rm -rf app 
