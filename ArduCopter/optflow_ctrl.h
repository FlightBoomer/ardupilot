#ifndef OPTFLOW_CTRL_H
#define OPTFLOW_CTRL_H

#include <cmath>
#include <stdio.h>
#include <stdarg.h>

#include "defines.h"
#include "config.h"

#include "GCS_Mavlink.h"
#include <GCS_MAVLink/GCS.h>

#include "../libraries/AP_AHRS/AP_AHRS.h"
#include "../libraries/AP_InertialNav/AP_InertialNav.h"         // ArduPilot Mega inertial navigation library
#include "../libraries/AP_InertialNav/AP_InertialNav_NavEKF.h"
#include "../libraries/AC_WPNav/AC_WPNav.h"                     // ArduCopter waypoint navigation library
#include "../libraries/AC_AttitudeControl/AC_PosControl.h"

#include "include/dt.hpp"

///
/// \brief The __optflow_ctrl class
/// 单位在输入的时候使用上一级函数的默认值
/// 输入以后在接收的函数全部转公制
/// 全部转为m和s和rad
/// 也全部以公制输出
/// 为了好算
class __optflow_ctrl {
public:

    __optflow_ctrl();

    ///
    /// 输入部分
    void set_AHRS_Data(double pitch_in, double roll_in, double yaw_in) {
        pitch = pitch_in;                                       // rad
        roll  = roll_in;
        yaw   = yaw_in;
    }

    void set_FlowRate(Vector2f in) {
        vel_r_raw_last = vel_r_raw_pre;
        vel_r_raw_pre  = vel_r_raw;
        vel_r_raw = in;
    }

    void set_BodyRate(Vector2f in) {
        vel_flow_last = vel_flow;                               // m/s
        vel_flow = in;
    }

    void set_HomePos(Vector3f in) {
        pos_offset = in;                                        // mm
        pos_offset.x /= 1000.0f;
        pos_offset.y /= 1000.0f;
        pos_offset.z /= 1000.0f;                                // mm->m
    }

    void set_GroundDistance(double in) {
        ground_distance = in *  cos(pitch) * cos(roll);         // mm, 飞行器角度修正，求出真正的对地距离
        ground_distance /= (double)1000.0f;                     // mm->m
    }

    ///
    /// 输出部分
    double get_GroundDistance()  { return ground_distance; }    // m
    Vector3f get_Position_ef()   { return pos_ef; }             // m
    Vector2f get_Velocity_ef()   { return vel_ef; }             // m/s
    Vector3f get_Position_ef_m() { return pos_ef_m; }           // m
    Vector2f get_Velocity_ef_m() { return vel_ef_m; }           // m/s


    int update();

private:

    double pitch, roll, yaw;                                          // in rads
    double ground_distance;                                           // mm

    Vector3f pos_offset;                                              // 起飞位置
    Vector3f pos, pos_last;                                           // 解算得到的当前位置
    Vector2f vel_r_raw, vel_r_raw_pre, vel_r_raw_last;                // 光流原始数据
    Vector2f vel_flow, vel_flow_last;                                 // 光流自身解算的机体系速度, m/s
    Vector2f vel, vel_last;

    Vector3f pos_ef;
    Vector2f vel_ef;

    Vector3f pos_m;                                                    // 光流自身结算出来的数据积分得到的位置
    Vector3f pos_ef_m;                                                 // 光流自身算出来的速度积分得到位置转NEU
    Vector2f vel_ef_m;                                                 // 光流自身算出来的速度转NEU

    uint32_t t, t_last;
    __dt dt;

    double roll_i, pitch_i;                                            // 角度积分，用来消除误差

};

class __optflow_inav_intf: public AP_InertialNav_NavEKF {              // __optflow_control_inav_interface
public:
    __optflow_inav_intf(AP_AHRS_NavEKF &ahrs) :
        AP_InertialNav_NavEKF(ahrs)
        {}

    void update_BasicData(AP_InertialNav_NavEKF in);

    void update_Flow(__optflow_ctrl in, bool is_use_flow_m);

private:

    double ground_dst, ground_dst_last;
    __dt dt;

};

/// 变量声明
extern __optflow_ctrl flow_ctrl;
extern __optflow_inav_intf *f_inav;
extern AC_PosControl       *of_pos_control;
extern AC_WPNav            *of_wp_nav;

#endif // OPTFLOW_CTRL_H
