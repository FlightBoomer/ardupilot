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

class __optflow_ctrl {
public:

    __optflow_ctrl();

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
        vel_flow_last = vel_flow;                               // cm/s
        vel_flow = in;
    }

    void set_HomePos(Vector3f in) {
        pos_offset = in;                                        // mm
    }

    void set_GroundDistance(double in) {
        ground_distance = in;                                   // mm
    }

    double get_GroundDistance()  { return ground_distance; }    // mm
    Vector3f get_Position_ef()   { return pos_ef; }             // mm
    Vector2f get_Velocity_ef()   { return vel_ef; }             // mm/s
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

    void update_BasicData(AP_InertialNav_NavEKF in) {
                _relpos_cm   = in.get_position();         // NEU
                _velocity_cm = in.get_velocity();         // NEU
                _pos_z_rate  = in.get_pos_z_derivative();

                /// 这两部分传不过来,用自己的AHRS读
                //_haveabspos  = _ahrs_ekf.get_position(_abspos);
                //_ahrs_ekf    = in._ahrs_ekf;
    }

    void update_Flow(__optflow_ctrl in, bool is_use_flow_m) {
        float x, y, z;
        Vector2f _vec2f;

        dt.update();

        if (!is_use_flow_m) {
            //
            _relpos_cm = in.get_Position_ef();
            _relpos_cm.x /= 10.0f, _relpos_cm.y /= 10.0f, _relpos_cm.z /= 10.0f;        // mm->cm

            //
            _vec2f = in.get_Velocity_ef();
            x = _vec2f.x, y = _vec2f.y;
            ground_dst_last = ground_dst;
            ground_dst = in.get_GroundDistance();
            z = (float)(ground_dst - ground_dst_last) / ((float)dt.get_dt_ms() / 1000.0f);     // mm/s
            z /= 10.0f;                                                                        // mm/s->cm/s

            _velocity_cm.x = x;
            _velocity_cm.y = y;
            _velocity_cm.z = z;
            _pos_z_rate    = z;

            _abspos.alt = (int32_t)_relpos_cm.z;       ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
            _abspos.lat = (int32_t)_relpos_cm.x;       ///< param 3 - Latitude * 10**7
            _abspos.lng = (int32_t)_relpos_cm.y;       ///< param 4 - Longitude * 10**7

            _haveabspos = true;
        }
        else {
            _relpos_cm = in.get_Position_ef_m();
            _relpos_cm.x *= 100.0f;                                                           // m->cm
            _relpos_cm.y *= 100.0f;
            _relpos_cm.z *= 100.0f;

            _vec2f = in.get_Velocity_ef_m() * 100.0f;
            ground_dst_last = ground_dst;
            ground_dst = in.get_GroundDistance() / (double)1000.0f;                           // mm->m
            z = (float)(ground_dst - ground_dst_last) / ((float)dt.get_dt_ms() / 1000.0f);    // m/s
            _velocity_cm.x = _vec2f.x;                                                        // cm/s
            _velocity_cm.y = _vec2f.y;
            _velocity_cm.z = z * 100.0f;                                                      // m/s->cm/s
            _pos_z_rate    = z * 100.0f;

            _abspos.alt = (int32_t)_relpos_cm.z;       ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
            _abspos.lat = (int32_t)_relpos_cm.x;       ///< param 3 - Latitude * 10**7
            _abspos.lng = (int32_t)_relpos_cm.y;       ///< param 4 - Longitude * 10**7

            _haveabspos = true;

            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "pos xy: %f, %f", (double)_relpos_cm.x, (double)_relpos_cm.y);
            //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "_relpos_cm.z: %f", (double)_relpos_cm.z);
            GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "gnd_dist_rate: %f", (double)_velocity_cm.z);
        }
    }

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
