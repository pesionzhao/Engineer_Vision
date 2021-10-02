#include     "Serial.h"

Serial::Serial()
{
}

Serial::~Serial()
{
}

//调用
//getShotSpeed(&vision_receive);
/*
 优先目标数字 - targetNum - 前三位（数字1 - 8——1英雄，2工程，345步兵，6哨兵，7前哨战，8基地）
 模式选择mode - 后五位（0 不处理，1 - 8留作模式选择，1为手动开火，2为自瞄，3为小符，4为大符）
 */
int INFOSIZE = 3;  //接收数据的大小
int num = 0;

int Serial::serialMode(float distance,float space,int now_mode, int& receive_color)
{
    uint16_t u_distance,u_space;

    u_distance = (int16_t)(distance * 100);
    u_space = (int16_t)(space * 100);

    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
    int err;               //返回调用函数的状态
    int len;
    int flag=0;

    //收到的数据
    uint8_t uart0_recv_buf[3];
    //串口接收的数据.
    uint8_t packages[INFOSIZE*3];

    vision_get vision_receive;

    //判断是否打开串口

    const char *dev[]  = {"/dev/ttyUSB0"};//THS1"};
    fd = open(dev[0],O_RDWR | O_NOCTTY ); //打开串口，返回文件描述符
    if(-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (0);
    }else{
        flag=1;//Seral open permit to trans
    }

    do{
        err = UART0_Init(fd,115200,0,8,1,'N');
    }while(FALSE == err || FALSE == fd);

    while(1)//serial fasong shuju
    {
        uint8_t uart0_send_buf[7];
        uart0_send_buf[0] = (uint8_t)0xFF;

        uart0_send_buf[1] = (u_distance >> 8) & 0xff;
        uart0_send_buf[2] = (u_distance) & 0xff;
        uart0_send_buf[3] = (u_space >> 8) & 0xff;
        uart0_send_buf[4] = (u_space) & 0xff;

        uart0_send_buf[5] = (uint8_t)now_mode;
        uart0_send_buf[6] = (uint8_t)0xBB;


        printf("%d", u_distance);

        if (flag)
        {
            len = UART0_Send(fd, uart0_send_buf, 7);
            if(len > 0)
            {
                printf("time send %d data successful\n",len);
            }else{
                printf("send data failed!\n");
            }

            int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));

//            recive_color = 1;
            if(len_receive > 0){
                printf("recive succsess");
            }else{
                printf("cannot recive \n");
            }

            //提取数据包
            bool readable = get_one_in_packages(uart0_recv_buf, packages);
            printf("readable : %d\n", readable);
            //解包——>得到一个完整的数据结构体vision_receive
            if (readable) {
//                printf("00000000.sof :%d\n",packages[0] );
//                printf("11111111.stone_color :%d\t", packages[1]);
//                printf("22222222.end :%d\n", packages[2]);
//                printf("33333333.sof :%d\n",packages[3] );
//                printf("11111111.stone_color :%d\t", packages[4]);
//                printf("22222222.end :%d\n", packages[5]);
//                printf("00000000.sof :%d\n",packages[6] );
//                printf("11111111.stone_color :%d\t", packages[7]);
//                printf("22222222.end :%d\n", packages[8]);


                buff_to_vision_receive(uart0_recv_buf,&vision_receive);
                printf("vision_receive.sof :%d\n", vision_receive.sof);
                printf("vision_receive.stone_color :%d\t", vision_receive.stone_color);
                printf("vision_receive.end :%d\n", vision_receive.end);
                if(vision_receive.stone_color == 0x00)
                    receive_color = 0;
                else if (vision_receive.stone_color == 0xFF)
                    receive_color = 1;

    //            if(vision_receive.stone_color == 0){
    //                printf("now stone_color is  yellow !!!!\n");
    //            }else if(vision_receive.stone_color == 1){
    //                printf("now stone_color is  white !!!!\n");
    //            }
                if (len_receive > 0)
                {
                    printf("\n receive %d data successful\n", len_receive);
                }else {
                    printf("\n receive data failed!\n");
                }

            }




            break;
        }
    }
    close(fd);
}


int Serial::UART0_Open(int fd,char*port)
{
    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
    if (fd<0)
    {
        perror("Can't Open Serial Port");
        return(FALSE);
    }
    //恢复串口为阻塞状态
    if(fcntl(fd, F_SETFL, FNDELAY) < 0)
    {
        printf("fcntl failed!\n");
        return(FALSE);
    }
    else
    {
        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
    }
    //测试是否为终端设备
    if(0 == isatty(STDIN_FILENO))
    {
        printf("standard input is not a terminal device\n");
        return(FALSE);
    }
    else
    {
        printf("isatty success!\n");
    }
    printf("fd->open=%d\n",fd);
    return fd;
}

int Serial::UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200,B57600, B38400,B19200,B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200, 57600,38400, 19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

    case 0 ://不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1 ://使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2 ://使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5    :
        options.c_cflag |= CS5;
        break;
    case 6    :
        options.c_cflag |= CS6;
        break;
    case 7    :
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return (FALSE);
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return (FALSE);
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return (FALSE);
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return (FALSE);
    }
    return (TRUE);
}

int Serial::UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err=0;
    //设置串口数据帧格式
    if (UART0_Set(fd,115200,0,8,1,'N') == FALSE)
    {
        return FALSE;
    }
    else
    {
        return  TRUE;
    }
}

int Serial::UART0_Recv(int fd, char *rcv_buf,int data_len)
{
//    int len,fs_sel;
//    fd_set fs_read;

//    struct timeval time;

//    FD_ZERO(&fs_read);
//    FD_SET(fd,&fs_read);    //如果fd == -1, FD_SET将在此阻塞

//    time.tv_sec = 1;
//    time.tv_usec = 0;

//    //串口的多路通信
//    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
//    printf("fs_sel = %d\n",fs_sel);



    if(1)//fs_sel)
    {
        int len = read(fd,rcv_buf,data_len);
        std::cout << "fd ===" << fd << std::endl;
        int flag = fcntl(fd,F_GETFL, 0);
        flag |= O_NONBLOCK;
        fcntl(fd,F_SETFL,flag);
        return len;
    }
    else
    {
        return FALSE;
    }
}

int Serial::UART0_Send(int fd, uint8_t *send_buf,int data_len)
{
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        //printf("send data is %d\n",send_buf);
        return len;
    }
    else
    {
        tcflush(fd,TCOFLUSH);
        return FALSE;
    }
}

bool Serial::get_one_in_packages(uint8_t * infoArray, uint8_t * packages)
{
    int ptr =0 ;
//    bool readable(true);

    for (ptr = 0; ptr < INFOSIZE * 3; ptr++) {
        if (packages[ptr] == 0x77 && packages[ptr+2] == 0x88)
        {
            std::cout<<"head and end is right!!!" << std::endl;
            for (int i = 0; i < INFOSIZE; i++)
                infoArray[i] = packages[ptr + i];
            return true;
        }
    }
    return false;
}


void Serial::buff_to_vision_receive(uint8_t * const uart0_recv_buf,Parse_vision_re receive_buf)
{
    receive_buf->sof = uart0_recv_buf[0]; //stone_color
    receive_buf->stone_color = uart0_recv_buf[1]; //stone_color
    receive_buf->end = uart0_recv_buf[2]; //stone_color
    //receive_buf->Bullet_speed = ((uint32_t)uart0_recv_buf[20] << 24) & ((uint32_t)uart0_recv_buf[21] << 16) & ((uint32_t)uart0_recv_buf[22] << 8) & (uint32_t)uart0_recv_buf[23];
}
