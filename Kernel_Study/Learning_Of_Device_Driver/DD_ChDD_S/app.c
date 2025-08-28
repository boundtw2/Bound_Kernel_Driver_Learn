#include <stdio.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>



int main()
{
	int fd;
	printf("------> led test <------\n");

	fd  = open("/dev/myled", O_RDWR);

	if(fd < 0)
	{
		printf("open /dev/myled-dev failed\n");
		return -1;
	}

	sleep(4);

	close(fd);

	return 0;

}



