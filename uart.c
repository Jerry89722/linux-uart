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
#include <signal.h>

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
		fd = open( "/dev/ttyS1", O_RDWR|O_NOCTTY);
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
	newtio.c_cc[VTIME] = 2;
	newtio.c_cc[VMIN] = 1;

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



/****************************************************************/
static struct flock tty_lock;

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

int cputemp_get(int fd)
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
	char tty_signal[64] = {};
	unsigned char power_signal = 0xff;
	int res = 0;
	while(1){
		res = read(fd, tty_signal, 64);
		
		if(!strncmp(tty_signal, "poweroff", 8)){
			if(fcntl(fd, F_SETLK, &tty_lock) == 0){
				write(fd, &power_signal, 1);
				close(fd);
			}
			system("poweroff -f");
			break;
		} else {
			memset(tty_signal, '\0', 64);
		}
	}
	return NULL;
}

void sighandler(int arg)
{
	//printf("get a signal = %d \n ", arg);
	char data;
	signalno = arg;
	return ;
}

char hddtemp_get(void)
{
	unsigned char temp = 0;
	char temp_str[12] = {};
	int status = 0;
	FILE* pf = NULL;
	
	//signal(SIGCHLD, SIG_IGN); 
	//popen调用产生的子进程退出后会发SIGHLD信号
	pf = popen("/bin/sh /tmp/run/hddtemp", "r");
	if(NULL == fgets(temp_str, 12, pf)) {
		return 0;
	}

	temp = atoi(temp_str);

	printf("hddtemp = %d \n", temp);
	pclose(pf);
	return temp;
}

void hddtemp_get_init(void){
//  获取硬盘温度脚本代码
char get_hddtempsh[] = "diskname=`cat /proc/partitions | grep \"sd[a-z]$\" | awk '{print $4}'`\n \
for i in $diskname; do\n \
	scsidevinfo=`find /sys/class/scsi_device/*/device/ -name $i`\n \
	ishdisk=`cat $scsidevinfo/removable`\n \
	if [ \"$ishdisk\" = 0 ]; then\n \
		disksymbol=`echo $scsidevinfo | awk -F '/' '{print $5}' | awk -F ':' '{print $1}'`\n \
		ls /proc/scsi/usb-storage/ 2>/dev/null | grep ^$disksymbol$\n \
		if [ $? != 0 ]; then\n \
			temp=$temp\" \"`lancehddtemp /dev/$i 2>/dev/null | awk -F ':' '{print $3}' | awk '{print $1}'`\n \
		fi\n \
	fi\n \
done\n \
disknum=`echo $temp | awk '{print NF}'`\n \
if [ $disknum = 1 ];then\n \
	echo $temp\n \
elif [ $disknum = 2 ]; then\n \
	res=`echo $temp | awk '{print $1 \" - \" $2}'`\n \
	res=`expr $res`\n \
	if [ $res -gt 0 ]; then\n \
		echo $temp | awk '{print $1}'\n \
	else\n \
		echo $temp | awk '{print $2}'\n \
	fi\n \
fi\n \
";

	int fd = open("/tmp/run/hddtemp", O_RDWR | O_CREAT, 777);
	if(fd < 0)
		perror("open"), exit(-1);
	int res = write(fd, get_hddtempsh, strlen(get_hddtempsh));
	if(res < 0)
		perror("write"), exit(-1);
	close(fd);
}

int main(int argc, char* argv[])
{
	int nwrite;
	unsigned char temp;
	int port = 2;
	
	signal(1, sighandler);   // 信号1用于恢复到正常读取硬盘和cpu的温度的流程中去
	signal(2, sighandler);  // 以下3个信号用于设置关机, 初始化, 恢复出厂设置的3种状态码值, 最后统一由串口发给mcu
	signal(3, sighandler);
	signal(4, sighandler);

	tty_lock.l_type = F_WRLCK;
	tty_lock.l_whence = SEEK_SET;
	tty_lock.l_start = 0;
	tty_lock.l_len = 10;
	
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
	
	hddtemp_get_init();  //产生硬盘温度获取的脚本
	char temp_hdd = 30;  //初始值赋为正常值
	unsigned char temp_cpu = 75; //初始值赋为正常值
	char flag_tempget = 0;
	for( ; ; ){
		if(signalno == 2)
			temp = 0xff; // 关机
		else if(signalno == 3)
			temp = 0xfe; // 初始化
		else if(signalno == 4)
			temp = 0xfd; // 恢复出厂设置
		else { //信号1时, 正常读取硬盘, cpu的温度
			if(flag_tempget++ >= 5){  // 每5s获取一次温度
				flag_tempget = 0;
				temp_cpu = (unsigned char)cputemp_get(fd_tempf);
				if((temp_hdd = hddtemp_get()) < 0)  // 防止环境温度极冷刚开机时硬盘温度小于0引发未知错误, 没硬盘时硬盘检测到的温度会是0
					temp_hdd = 0;
				temp_hdd = (75*10 + (temp_hdd - 30) * 16) / 10;
			
				temp = temp_cpu > temp_hdd ? temp_cpu : temp_hdd;  //30~55 75~115

				// printf("temp = %d, temp_hdd = %d, temp_cpu = %d \n", temp, temp_hdd, temp_cpu);  //没硬盘时温度temp_hdd = 27
			}
		}
		//printf("signalno = %d \n", signalno);
		if(fcntl(fd_tty, F_SETLK, &tty_lock) == 0){
			nwrite=write(fd_tty, &temp, 1);//写串口
			lseek(fd_tty, SEEK_SET, 0);
			tty_lock.l_type = F_UNLCK;
			fcntl(fd_tty, F_SETLK, &tty_lock);
		}
		sleep(1);
	}
	close(fd_tempf);
	close(fd_tty);
	return;
}

