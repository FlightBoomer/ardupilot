#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <string.h>

// https://blog.csdn.net/daiyudong2020/article/details/70039409

#define MYPORT 8888

int sock;

#define ERR_EXIT(m) \
    do { \
    perror(m); \
    exit(EXIT_FAILURE); \
    } while (0)

bool echo_ser(int sock)
{


    char recvbuf[1024] = {0};
    struct sockaddr_in peeraddr;
    socklen_t peerlen;
    int n;

    //while (1)
    //{
        peerlen = sizeof(peeraddr);
        memset(recvbuf, 0, sizeof(recvbuf));
        n = recvfrom(sock, recvbuf, sizeof(recvbuf), 0,
                     (struct sockaddr *)&peeraddr, &peerlen);
        if (n <= 0)
        {

            if (errno == EINTR)
                return -1; // continue


            //ERR_EXIT("recvfrom error");
        }
        else if(n > 0)
        {
            printf("接收到的数据：%s\n",recvbuf);
            sendto(sock, recvbuf, n, 0,
                   (struct sockaddr *)&peeraddr, peerlen);
            printf("回送的数据：%s\n",recvbuf);
        }
    //}
    //close(sock);
}

int init_orb_SLAM_udp(void)
{
    if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket error");

    // 设置超时
    struct timeval timeout;
    timeout.tv_sec = 1;//秒
    timeout.tv_usec = 0;//微秒
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        perror("setsockopt failed:");
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(MYPORT);
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    struct timeval timeOut;
    timeOut.tv_sec = 0;                 //设置5s超时
    timeOut.tv_usec = 70000;
    if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeOut, sizeof(timeOut)) < 0)
    {
        printf("time out setting failed\n");
    }

    printf("监听%d端口\n",MYPORT);
    if (bind(sock, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        ERR_EXIT("bind error");

    echo_ser(sock);

    return 0;
}
