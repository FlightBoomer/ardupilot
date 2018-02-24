#include "Copter.h"
#include <opencv2/opencv.hpp>
#include "include/lidar_driver.hpp"
#include "include/icp_interface.hpp"


using namespace cv;

__lidar_driver	lidar;
__icp			_icp;
bool is_lidar_online     = false;
bool is_lidarpos_updated = false;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    int status;
    status = lidar.init();
    if (status == SUCCESS) {
        is_lidar_online = true;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "rplidar is online..");
    } else {
        is_lidar_online = false;
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "rplidar is offline..");
    }
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here

    ///
    /// 激光雷达扫描
    const  int is_ok_thereshold = 2;		// 采样几次后处理，这个值可以理解为后续的样本点，样本点越多，后续的精度增加非常多， 但是样本点越多，实时性越差
    static int is_ok_times      = 0;		// 执行状态标识
    u_result  op_result = -1;

    if (is_lidar_online) {
        rplidar_response_measurement_node_t nodes[360 * 2];
        size_t   count = _countof(nodes);
        bool is_express = false;
        bool is_4k_mode = false;

        if (is_ok_times == 0) {
            /// 扫描开始
            lidar.Data.clear();
            is_ok_times++;
        } else if (is_ok_times < is_ok_thereshold) {
            /// 正在扫描，因为apm不会自动释放正在执行的任务, 所以千万别在任务函数里写长时间的等待
            op_result = lidar.drv->grabScanData(nodes, count);
            if (IS_OK(op_result)) {
                float frequency = 0;
                // 读取数据
                lidar.drv->getFrequency(is_express, count, frequency, is_4k_mode);
                lidar.scanData(nodes, count, frequency, false);
                is_ok_times++;
            }
        } else {
            /// 完成扫描
            // 作图
            //draw(Img, Data, (char *)"Raw", is_show);
            _icp.run(lidar.Data, false);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "icp: dx: %f, dy: %f", _icp.dx, _icp.dy);

            is_ok_times = 0;
            is_lidarpos_updated = true;
            waitKey(30);
        }
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
