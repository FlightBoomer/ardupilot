#ifndef LIDAR_DRIVER_H
#define LIDAR_DRIVER_H

//#define WIN32

#include "config.h"
#include "misc_tools.hpp"

#include <../libraries/rplidar_sdk/rplidar.h>       // #include <rplidar_sdk/rplidar.h>
#include <signal.h>

#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>

#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))       // linux没有这个宏，只能自己定义

using namespace std;
using namespace cv;
using namespace rp::standalone::rplidar;

void ctrlc(int);

// 注意:这个驱动是为了RplidarA2设计的
// 如果是A1，角度间隔是1°而非0.5°，需要额外调整
class __lidar_driver {
public:
    __lidar_driver();

    vector<__scandot> laserData;
    double            laserArray[720];					// 0°开始, 0.5°间隔

    int init();

    int grab_ScanData();                                // 更新扫描数据

    int draw(Mat &dst, vector<__scandot> data, char window_name[], bool is_show);
    int draw(Mat &dst, double data[], char window_name[], bool is_show);

private:

    char	* opt_com_path = NULL;
    _u32 baudrateArray[2] = { 115200, 256000 };
    _u32 opt_com_baudrate = 0;

    RPlidarDriver * drv;

    rplidar_response_device_info_t devinfo;

    bool checkRPLIDARHealth(RPlidarDriver * drv);

};

#endif // LIDAR_DRIVER_H
