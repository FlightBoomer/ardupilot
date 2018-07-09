#ifndef ORB_SLAM_UDP_H
#define ORB_SLAM_UDP_H

#include "user_config.h"
#include "orb_slam_interface.h"
#include "dt.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <string.h>

#define Rx_Len 512

class __orb_slam_udp : public __orb_slam {
public:
    __orb_slam_udp() : __orb_slam() {

    }

    int init() override;

    int update() override;

protected:

    int socket_fd;
    char Rx[Rx_Len];

    struct timeval timeout;

    __dt dt;

    int set_Timeout();

    int recv_Data();

    int refine_Data();

    void calc_PosRate(double _dt_s) override;

    void calc_AngularRate(double _dt_s) override;

};

#endif // ORB_SLAM_UDP_H
