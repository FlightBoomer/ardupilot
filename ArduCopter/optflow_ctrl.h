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
        ground_distance = in * cos(pitch) * cos(roll);          // mm, 飞行器角度修正，求出真正的对地距离, 本来可以均值滤波，但是考虑到均值滤波加完对微分影响很大所以不加了
        ground_distance /= (double)1000.0f;                     // mm->m
    }

    ///
    /// 输出部分
    double get_GroundDistance()  { return ground_distance; }    // m
    Vector3f get_Position_ef()   { return pos_ef; }             // m
    Vector2f get_Velocity_ef()   { return vel_ef; }             // m/s
    Vector3f get_Position_ef_m() { return pos_ef_m; }           // m
    Vector2f get_Velocity_ef_m() { return vel_ef_m; }           // m/s
    Vector2f get_Velocity_m()    { return vel_flow; }           // m/s


    int update();

private:

    double pitch, roll, yaw;                                          // in rads
    double ground_distance;                                           // m

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

class __pos_ctrl_body: public AC_PosControl {
public:
    __pos_ctrl_body(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                    const AP_Motors& motors, AC_AttitudeControl& attitude_control,
                    AC_P& p_pos_z, AC_P& p_vel_z, AC_PID& pid_accel_z,
                    AC_P& p_pos_xy, AC_PI_2D& pi_vel_xy) :
    AC_PosControl(ahrs, inav,
                  motors, attitude_control,
                  p_pos_z, p_vel_z, pid_accel_z,
                  p_pos_xy, pi_vel_xy) {

    }

    void set_current_vel_body_xy(float vx_in, float vy_in) {
        vel_raw_x = vx_in;
        vel_raw_y = vy_in;
    }

    void set_vel_target_xy(float vx_in, float vy_in) {
        vel_tgt_x = vx_in;
        vel_tgt_y = vy_in;
    }

    void update_vel_controller_body_xy(float ekfNavVelGainScaler) {
        // capture time since last iteration
        uint32_t now = AP_HAL::millis();
        float dt = (now - _last_update_xy_ms)*0.001f;

        // call xy controller
        if (dt >= get_dt_xy()) {
            // sanity check dt
            if (dt >= 0.2f) {
                dt = 0.0f;
            }

            // check for ekf xy position reset
            check_for_ekf_xy_reset();

            // check if xy leash needs to be recalculated
            //calc_leash_length_xy();

            // apply desired velocity request to position target
            //desired_vel_to_pos(dt);

            // run position controller's position error to desired velocity step
            pos_to_rate_xy(XY_MODE_POS_LIMITED_AND_VEL_FF, dt, ekfNavVelGainScaler);

            // run velocity to acceleration step
            rate_to_accel_xy(dt, ekfNavVelGainScaler);

            // run acceleration to lean angle step
            accel_to_lean_angles_body(dt, ekfNavVelGainScaler, false);

            // update xy update time
            _last_update_xy_ms = now;
        }

        // update altitude target
        set_alt_target_from_climb_rate_ff(_vel_desired.z, _dt, false);

        // run z-axis position controller
        update_z_controller();
    }

protected:

    float vel_raw_x, vel_raw_y;
    float vel_tgt_x, vel_tgt_y;

    void accel_to_lean_angles_body(float dt, float ekfNavVelGainScaler, bool use_althold_lean_angle)
    {
        float accel_total;                          // total acceleration in cm/s/s
        float accel_right, accel_forward;
        float lean_angle_max = _attitude_control.lean_angle_max();
        float accel_max = POSCONTROL_ACCEL_XY_MAX;

        // limit acceleration if necessary
        if (use_althold_lean_angle) {
            accel_max = MIN(accel_max, GRAVITY_MSS * 100.0f * tanf(ToRad(constrain_float(_attitude_control.get_althold_lean_angle_max(),1000,8000)/100.0f)));
        }

        // scale desired acceleration if it's beyond acceptable limit
        accel_total = norm(_accel_target.x, _accel_target.y);
        if (accel_total > accel_max && accel_total > 0.0f) {
            _accel_target.x = accel_max * _accel_target.x/accel_total;
            _accel_target.y = accel_max * _accel_target.y/accel_total;
            _limit.accel_xy = true;     // unused
        } else {
            // reset accel limit flag
            _limit.accel_xy = false;
        }

        // reset accel to current desired acceleration
        if (_flags.reset_accel_to_lean_xy) {
            _accel_target_jerk_limited.x = _accel_target.x;
            _accel_target_jerk_limited.y = _accel_target.y;
            _accel_target_filter.reset(Vector2f(_accel_target.x, _accel_target.y));
            _flags.reset_accel_to_lean_xy = false;
        }

        // apply jerk limit of 17 m/s^3 - equates to a worst case of about 100 deg/sec/sec
        float max_delta_accel = dt * _jerk_cmsss;

        Vector2f accel_in(_accel_target.x, _accel_target.y);
        Vector2f accel_change = accel_in-_accel_target_jerk_limited;
        float accel_change_length = accel_change.length();

        if(accel_change_length > max_delta_accel) {
            accel_change *= max_delta_accel/accel_change_length;
        }
        _accel_target_jerk_limited += accel_change;

        // lowpass filter on NE accel
        _accel_target_filter.set_cutoff_frequency(MIN(_accel_xy_filt_hz, 5.0f*ekfNavVelGainScaler));
        Vector2f accel_target_filtered = _accel_target_filter.apply(_accel_target_jerk_limited, dt);

        // rotate accelerations into body forward-right frame
        //accel_forward = accel_target_filtered.x*_ahrs.cos_yaw() + accel_target_filtered.y*_ahrs.sin_yaw();
        //accel_right = -accel_target_filtered.x*_ahrs.sin_yaw() + accel_target_filtered.y*_ahrs.cos_yaw();
        /// 自增加，这里不做旋转，这里改称直接写入，因为这个类输入的数据是机体的
        accel_forward = accel_target_filtered.x;
        accel_right   = accel_target_filtered.y;

        // update angle targets that will be passed to stabilize controller
        _pitch_target = constrain_float(atanf(-accel_forward/(GRAVITY_MSS * 100))*(18000/M_PI),-lean_angle_max, lean_angle_max);
        float cos_pitch_target = cosf(_pitch_target*M_PI/18000);
        _roll_target = constrain_float(atanf(accel_right*cos_pitch_target/(GRAVITY_MSS * 100))*(18000/M_PI), -lean_angle_max, lean_angle_max);
    }


};

/// 变量声明
extern __optflow_ctrl flow_ctrl;

extern __optflow_inav_intf *f_inav;
extern AC_PosControl       *of_pos_control;
extern __pos_ctrl_body     *of_xy;
//extern AC_WPNav          *of_wp_nav;

#endif // OPTFLOW_CTRL_H
