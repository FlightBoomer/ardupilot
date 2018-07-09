#ifndef __ORB_SLAM_Interface_HPP
#define __ORB_SLAM_Interface_HPP

#include "user_config.h"

class __orb_slam {
public:
    __orb_slam() {

    }

    ~__orb_slam() {

    }

    virtual int init() {
        return -1;
    }

    virtual int update() {
        return -1;
    }

    __vec3f get_Pos() { return pos; }
    __vec3f get_PosRate() { return pos_rate; }

    __imu   get_Cam_Attitude() { return imu; }
    __imu   get_Cam_Att_Rate() { return imu_rate; }

protected:

    __imu   imu, imu_last, imu_rate;
    __vec3f pos, pos_last, pos_rate;

    virtual void calc_PosRate(double _dt_s) {

    }

    virtual void calc_AngularRate(double _dt_s) {

    }

};




#endif  // __ORB_SLAM_Interface_HPP
