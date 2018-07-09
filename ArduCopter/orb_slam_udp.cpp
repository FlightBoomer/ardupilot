#include "user_include/orb_slam_udp.h"

// https://blog.csdn.net/daiyudong2020/article/details/70039409

#define MYPORT 8888

using namespace  std;


#define ERR_EXIT(m) \
    do { \
    perror(m); \
    exit(EXIT_FAILURE); \
    } while (0)

int __orb_slam_udp::update() {
    int stat;
    stat = recv_Data();

    if (stat == 0) {
        refine_Data();

        dt.update();
        double t_s = dt.get_dt_s();     // time in seconds

        cout << "dt:" << t_s << endl;

        calc_PosRate(t_s);
        calc_AngularRate(t_s);
    }

    return stat;

}

int __orb_slam_udp::init() {

    if ((socket_fd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
        ERR_EXIT("socket error");

    if (set_Timeout() < 0)
        return -1;

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(MYPORT);
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

    printf("监听%d端口\n",MYPORT);
    if (bind(socket_fd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        ERR_EXIT("bind error");

    return 0;
}

int __orb_slam_udp::set_Timeout() {

    // 设置超时
    //struct timeval timeout;
    timeout.tv_sec = 0;//秒
    timeout.tv_usec = 70000;//微秒
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        perror("setsockopt failed:");
        return -1;
    }
    return 0;
}

int __orb_slam_udp::recv_Data() {

    struct sockaddr_in peeraddr;
    socklen_t peerlen;
    int n;

    peerlen = sizeof(peeraddr);
    memset(Rx, 0, sizeof(Rx));
    n = recvfrom(socket_fd, Rx, sizeof(Rx), 0,
                     (struct sockaddr *)&peeraddr, &peerlen);
    if (n <= 0) {
        if (errno == EINTR)
            return -1; // continue

            //ERR_EXIT("recvfrom error");
    }
    else if(n > 0) {
        //printf("接收到的数据：%s\n",Rx);
        //sendto(socket_fd, recvbuf, n, 0,
        //      (struct sockaddr *)&peeraddr, peerlen);
        //printf("回送的数据：%s\n",Rx);
        return 0;
    }
    return -1;
}

int __orb_slam_udp::refine_Data() {

    float _pitch, _roll, _yaw;
    float _x, _y, _z;

    sscanf(Rx, "roll: %f pitch: %f yaw: %f x: %f y: %f z: %f  ",
           &_roll, &_pitch, &_yaw,
           &_x,    &_y,     &_z);

    imu_last = imu;
    pos_last = pos;

    imu.roll = _roll; imu.pitch = _pitch; imu.yaw = _yaw;
    pos.x    = _x;    pos.y     = _y;     pos.z   = _z;

    cout << "x: " << pos.x << " y: " << pos.y << endl;

    return 0;
}

void __orb_slam_udp::calc_PosRate(double _dt_s) {

    pos_rate.x = (pos.x - pos_rate.x) / _dt_s;
    pos_rate.z = (pos.y - pos_rate.y) / _dt_s;
    pos_rate.y = (pos.z - pos_rate.z) / _dt_s;

}

void __orb_slam_udp::calc_AngularRate(double _dt_s) {

    imu_rate.roll  = (imu.roll  - imu_last.roll)  / _dt_s;
    imu_rate.pitch = (imu.pitch - imu_last.pitch) / _dt_s;
    imu_rate.yaw   = (imu.yaw   - imu_last.yaw)   / _dt_s;

}
