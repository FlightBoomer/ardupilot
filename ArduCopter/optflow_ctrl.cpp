#include "optflow_ctrl.h"

// https://blog.csdn.net/u010720661/article/details/54019410
// https://blog.csdn.net/u011913159/article/details/79616492
// https://blog.csdn.net/u010720661/article/details/54019410

__optflow_ctrl flow_ctrl;

AP_InertialNav *f_inav;
AC_PosControl *of_pos_control;
AC_WPNav *of_wp_nav;

__optflow_ctrl::__optflow_ctrl() {

    pitch = 0, roll = 0, yaw = 0;

    pos_offset.x     = pos_offset.y = 0;
    pos.x = pos.y    = 0;
    pos_last.x       = pos_last.y = 0;

    vel_r_raw.x      = vel_r_raw.y      = 0;
    vel_r_raw_pre.x  = vel_r_raw_pre.y  = 0;
    vel_r_raw_last.x = vel_r_raw_last.y = 0;

    vel_flow.x       = vel_flow.y       = 0;
    vel_flow_last.x  = vel_flow_last.y  = 0;

    vel.x            = vel.y            = 0;
    vel_last.x       = vel_last.y       = 0;

    roll_i = pitch_i = 0;

    t =  t_last = 0;
}

int __optflow_ctrl::update() {

    ///
    /// \brief lean_angle_threshold
    /// y(front)
    /// ↑
    /// o →x(right)

    const static double lean_angle_threshold = 30 * M_PI / 180.0f;        // 30 degree
    /// 镜头参数, https://www.axis.com/zh-cn/products/lenses/lens-m12-16-mm
    const static double scope_length = 16;                                // 镜头长, mm
    const static double flow_img_heigth = 64;                             // pixel
    const static double flow_img_width  = 64;
    const static double ccd_width  = 4.51;                                // mm
    const static double ccd_heigth = 2.88;                                // mm

    /// 如果角度过大得到的光流数据是不准确的，所以不予以计算
    if (fabs(pitch) >= lean_angle_threshold || fabs(roll) >= lean_angle_threshold)
        return -1;

    /// 更新时间
    dt.update();

    /// 均值滤波
    double vr_x, vr_y;
    vr_x = (vel_r_raw.x + vel_r_raw_pre.x + vel_r_raw_last.x) / 3.0f;
    vr_y = (vel_r_raw.y + vel_r_raw_pre.y + vel_r_raw_last.y) / 3.0f;

    /// 摄像机像素移动结合视场角转换为速度
    /// 相似三角形算出
    double vx_gnd, vy_gnd;
    vx_gnd = (vr_x / flow_img_width)  * ccd_width  * (ground_distance / scope_length);
    vy_gnd = (vr_y / flow_img_heigth) * ccd_heigth * (ground_distance / scope_length);

    /// 速度积分变为位移
    pos_last = pos;
    pos.x += (float)vx_gnd;
    pos.y += (float)vy_gnd;

    /// IMU得到的欧拉角角度积分变成角量，用来抵消角度运动时造成的误差
    roll_i += roll;
    pitch_i += pitch;

    /// 角运动乘以高度转线运动
    //double x_int, y_int;
    //x_int = roll_i  * ground_distance;
    //y_int = pitch_i * ground_distance;


    /// 积分数据融合
    //pos_x -= x_int;
    //pos_y -= y_int;

    /// 位移LPF
    pos.x = 0.8 * pos.x + 0.2 * pos_last.x;
    pos.y = 0.8 * pos.y + 0.2 * pos_last.y;

    /// 微分得到速度
    vel_last = vel;
    vel.x = (pos.x - pos_last.x) / (float)dt.get_dt_ms();
    vel.y = (pos.y - pos_last.y) / (float)dt.get_t_all_ms();

    /// 速度LPF
    vel.x = 0.8 * vel.x + 0.2 * vel_last.x;
    vel.y = 0.8 * vel.y + 0.2 * vel_last.y;

    /// 对机体转对地
    pos_ef.x = pos.x * (float)cos(yaw) - pos.y * (float)sin(yaw);  // north
    pos_ef.y = pos.x * (float)sin(yaw) + pos.y * (float)cos(yaw);  // east

    vel_ef.x = vel.x * (float)cos(yaw) - vel.y * (float)sin(yaw);
    vel_ef.y = vel.x * (float)cos(yaw) - vel.y * (float)sin(yaw);


    return 0;
}

