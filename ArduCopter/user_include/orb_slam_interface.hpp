#ifndef __ORB_SLAM_Interface_HPP
#define __ORB_SLAM_Interface_HPP

#include "user_config.h"

class __orb_slam {
public:
    __orb_slam() {

    }

    ~__orb_slam() {

    }

    virtual int init();

    virtual int update();

    __vec3f get_Pos() { return pos; }
    __vec3f get_PosRate() { return pos_rate; }

    __imu   get_Cam_Attitude() { return imu; }
    __imu   get_Cam_Att_Rate() { return imu_rate; }

protected:

    __imu   imu, imu_rate;
    __vec3f pos, pos_rate;

};




#endif  // __ORB_SLAM_Interface_HPP
