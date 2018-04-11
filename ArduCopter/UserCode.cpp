#include "Copter.h"
#include "include/config.h"
#include "include/lidar_driver.h"


using namespace cv;

__lidar_driver *lidar;

bool is_lidar_online;

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

    if (is_lidar_online == true) {

        status = lidar->grab_ScanData();
        if (status == __SUCCEEDED) {
            Mat dst;
            lidar->draw(dst, lidar->laserArray, "raw", true);
        }
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
