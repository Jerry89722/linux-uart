#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>

/*打开串口函数*/
int open_port(int fd,int comport)
{
	char *dev[]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2"};
	long vdisable;
	if (comport==1)//串口 1
	{
		fd = open( "/dev/ttyS0", O_RDWR|O_NOCTTY|O_NDELAY);
		if (-1 == fd){
			perror("Can't Open Serial Port");
			return(-1);
		}

	}	
	else if(comport==2)//串口 2
	{
		fd = open( "/dev/ttyS1", O_RDWR|O_NOCTTY|O_NDELAY);
		if (-1 == fd){
			perror("Can't Open Serial Port");
			return(-1);
		}
	}

	else if (comport==3)//串口 3
	{
		fd = open( "/dev/ttyS2", O_RDWR|O_NOCTTY|O_NDELAY);
		if (-1 == fd){
			perror("Can't Open Serial Port");
			return(-1);
		}
	}
	/*恢复串口为阻塞状态*/

	if(fcntl(fd, F_SETFL, 0)<0)
		printf("fcntl failed!\r\n");

	else
		printf("fcntl=%d\r\n",fcntl(fd, F_SETFL,0));

	/*测试是否为终端设备*/

	if(isatty(STDIN_FILENO)==0)

		printf("standard input is not a terminal device\r\n");

	else
		printf("isatty success!\r\n");

	printf("fd-open=%d\r\n",fd);
	return fd;
}

int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{

	struct termios newtio,oldtio;

	/*保存测试现有串口参数设置,在这里如果串口号等出错,会有相关的出错信息*/
	if ( tcgetattr( fd,&oldtio) != 0) {
		perror("SetupSerial 1");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	
	/*步骤一,设置字符大小*/
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	
	/*设置停止位*/
	switch( nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	}
	/*设置奇偶校验位*/

	switch( nEvent )
	{
	case 'O': //奇数
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': //偶数
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N': //无奇偶校验位
		newtio.c_cflag &= ~PARENB;
		break;
	}

	/*设置波特率*/

	switch( nSpeed )
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	}

	/*设置停止位*/
	if( nStop == 1 )
		newtio.c_cflag &= ~CSTOPB;

	else if ( nStop == 2 )
		newtio.c_cflag |= CSTOPB;

	/*设置等待时间和最小接收字符*/
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	/*处理未接收字符*/
	tcflush(fd,TCIFLUSH);

	/*激活新配置*/
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)
	{
		perror("com set error");
		return -1;
	}
	//printf("set done!\r\n");
	return 0;
}

int char2temp(char* p_temp)
{
	int temp;
	//printf("len = %d \r\n", strlen(p_temp));
	if(strlen(p_temp) == 6)
		temp = (p_temp[0]-'0') * 10 + (p_temp[1] - '0');
	else if(strlen(p_temp) == 7)
		temp = (p_temp[0] - '0') * 100 + (p_temp[1] - '0') * 10 + (p_temp[2] - '0');
	else
		return -1;
	//printf("temp = %d \r\n", temp);
	return temp;
}

int tempf_open(void)
{
	int fd = open("/sys/class/hwmon/hwmon0/temp1_input", O_RDONLY);
	if(fd < 0)
	{
		perror("temp file open fail: "); 
		return -1;
	}
	return fd;
}

int get_temp(int fd)
{
	char temp_char[10] = {};

	int res = read(fd, temp_char, 10);
	if(res < 0)
	{
		perror("temp file read fail: "); 
		return -1;
	}

	res = lseek(fd, SEEK_SET, 0);

	return char2temp(temp_char);
}

static int fd_tty;
static int signalno = 0;
void* tty_rcv(void* fd_tty)
{
	int fd = *((int*)fd_tty);
	char tty_signal[10] = {};
	unsigned char power_signal = 0xff;
	while(1){
		int res = read(fd, tty_signal, 10);
		lseek(fd, SEEK_SET, 0);
		
		if(!strncmp(tty_signal, "poweroff", 8)){
			printf("poweroff \n");
			write(fd, &power_signal, 1);
			//kill(getpid(), 2);
			signalno = 2;
			system("poweroff -f");
			break;
		} else {
			memset(tty_signal, '\0', 10);
		}
	}
	return NULL;
}

void sighandler(int arg)
{
	printf("get a signal = %d \n ", arg);
	char data;
	signalno = arg;
	return ;
}

int main(int argc, char* argv[])
{
	int nwrite;
	unsigned char temp;
	int port = 2;
	
	signal(1, sighandler);
	signal(2, sighandler);
	signal(3, sighandler);
	signal(4, sighandler);

	if((fd_tty=open_port(fd_tty, port))<0){//打开串口
		perror("open_port error");
		return;
	}
	if((set_opt(fd_tty,115200,8,'N',1))<0){//设置串口
		perror("set_opt error");
		return;
	}

	int fd_tempf = tempf_open();
	pthread_t tid;
	pthread_create(&tid, NULL, tty_rcv, (void*)&fd_tty);

	pthread_detach(tid);
	for( ; ; ){
		//printf("temperature = %d \r\n", temp);
		if(signalno == 2)
			temp = 0xff; // 关机
		else if(signalno == 3)
			temp = 0xfe; // 初始化
		else if(signalno == 4)
			temp = 0xfd; // 恢复出厂设置
		else
			temp = (unsigned char)get_temp(fd_tempf);
		
		nwrite=write(fd_tty, &temp,1);//写串口
		lseek(fd_tty, SEEK_SET, 0);
		sleep(1);
	}
	close(fd_tempf);
	close(fd_tty);
	return;
}

