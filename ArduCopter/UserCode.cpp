#include "Copter.h"
#include "include/config.h"
#include "include/icp_interface.h"
#include "include/lidar_driver.h"
#include "include/og_mapping.h"


using namespace cv;

__lidar_driver *lidar;
GridMapping *map_laser;
GridMapping *map_local;
__icp *icp_local_global;

bool is_lidar_online;
double x_esti = 0, y_esti = 0;
double robotTheta = 0;

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    int status;

    lidar = new __lidar_driver;
    status = lidar->init();

    if (status == __SUCCEEDED) {
        is_lidar_online = true;

        map_laser = new GridMapping(0, 2.2, -2.2, 200, 0.5, 13500, 0, 0);

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
    int status;
    static int i = 1;

    ///
    /// 更新欧拉角
    robotTheta = ahrs.yaw * 180.0f / PI;
    while (robotTheta < 0)      // 角度归一化至0~360
        robotTheta += 360.0f;
    while (robotTheta >= 360.0f)
        robotTheta -= 360.0f;

    ///
    /// 更新SLAM
    if (is_lidar_online == true) {

        status = lidar->grab_ScanData();
        if (status == __SUCCEEDED) {
            Mat dst;
            lidar->draw(dst, lidar->laserArray, "raw", true);

            // 前5帧只要塞入正确的数据即可
            if (i <= 5) {
                map_laser->updateGridMap_Laser(0, 0, robotTheta * (CV_PI / 180), lidar->laserArray);		// 更新整图
                map_laser->showGridMap("laser&&cv2");
                map_laser->updateEdge(3.0f);

                i++;
                return;
            }

            // 更新子图
            if (i % 3 == 0) {
                delete map_local;
                map_local = new GridMapping(0, 2.2, -2.2, 200, 0.5, 80000, 0, 0);
            }
            map_local->updateGridMap_Laser(x_esti, y_esti, robotTheta * (CV_PI / 180), lidar->laserArray);
            map_local->showGridMap("laser&&local");
            map_local->updateEdge(2.5f);

            if (i % 3 == 0) {

                // icp匹配
                if (map_local->edge.size()) {
                    icp_local_global->set_Pt_ref(map_laser->edge);
                    icp_local_global->set_Pt_to(map_local->edge);
                    icp_local_global->run();
                    icp_local_global->show_DataResult();

                    double icp_x, icp_y;
                    icp_local_global->get_Pos_Shifted(icp_x, icp_y);
                    x_esti += icp_x * 100.0f;
                    y_esti += icp_y * 100.0f;
                }
            }

            // 更新整图
            map_laser->updateGridMap_Laser(x_esti, y_esti, robotTheta * (CV_PI / 180), lidar->laserArray);		// 更新整图
            map_laser->showGridMap("laser&&cv2");
            map_laser->updateEdge(3.0f);
        }
        i++;
        waitKey(1);
    }// is_lidar_online
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
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
