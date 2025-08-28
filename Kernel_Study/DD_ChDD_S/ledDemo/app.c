
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>


#define LED1 0x1 
#define LED2 0x2 
#define LED3 0x3 
#define LED4 0x4 


#define LED_ON 	0xe
#define LED_OFF	0xf


int main(int argc,char **argv)
{
	int fd,led;
	fd = open("/dev/myled",O_RDWR);			//sys_open
	if(fd < 0)
	{
		printf("open dev file fail\n");
		return -1;
	}

	read(fd, &led, 4);
	printf("led=0x%x\n",led);

	while(1)
	{
		ioctl(fd, LED_OFF, LED1);
		sleep(1);
		ioctl(fd, LED_ON, LED1);
		sleep(1);
		ioctl(fd, LED_OFF, LED2);
		sleep(1);
		ioctl(fd, LED_ON, LED2);
		sleep(1);
		ioctl(fd, LED_OFF, LED3);
		sleep(1);
		ioctl(fd, LED_ON, LED3);
		sleep(1);
		//ioctl(fd, LED_OFF, LED4);
		//sleep(1);
		//ioctl(fd, LED_ON, LED4);
		//sleep(1);

	}

	close(fd);
}



