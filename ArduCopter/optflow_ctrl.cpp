#include "optflow_ctrl.h"



/*

    // 参考资料
        https://blog.csdn.net/u010720661/article/details/54019410
        https://blog.csdn.net/u011913159/article/details/79616492
        https://blog.csdn.net/u010720661/article/details/54019410

    Px4flow的代码内光流数据的定义                                              http://pixhawk.org/modules/px4flow
        typedef struct i2c_frame
        {
            uint16_t frame_count;// counts created I2C frames [#frames]
            int16_t pixel_flow_x_sum;// latest x flow measurement in pixels*10 [pixels]
            int16_t pixel_flow_y_sum;// latest y flow measurement in pixels*10 [pixels]
            int16_t flow_comp_m_x;// x velocity*1000 [meters/sec]
            int16_t flow_comp_m_y;// y velocity*1000 [meters/sec]
            int16_t qual;// Optical flow quality / confidence [0: bad, 255: maximum quality]
            int16_t gyro_x_rate; // latest gyro x rate [rad/sec]
            int16_t gyro_y_rate; // latest gyro y rate [rad/sec]
            int16_t gyro_z_rate; // latest gyro z rate [rad/sec]
            uint8_t gyro_range; // gyro range [0 .. 7] equals [50 deg/sec .. 2000 deg/sec]
            uint8_t sonar_timestamp;// time since last sonar update [milliseconds]
            int16_t ground_distance;// Ground distance in meters*1000 [meters]. Positive value: distance known. Negative value: Unknown distance
        } i2c_frame;

    APM内的读取代码
        state.surface_quality = frame.qual;
        state.flowRate = Vector2f(frame.pixel_flow_x_integral * flowScaleFactorX,
                                  frame.pixel_flow_y_integral * flowScaleFactorY) * 1.0e-4 * integralToRate;
        state.bodyRate = Vector2f(frame.gyro_x_rate_integral, frame.gyro_y_rate_integral) * 1.0e-4 * integralToRate;
        state.ground_distance = (double)frame.ground_distance;

*/

__optflow_ctrl flow_ctrl;

__optflow_inav_intf *f_inav;
AC_PosControl       *of_pos_control;
AC_WPNav            *of_wp_nav;

__optflow_ctrl::__optflow_ctrl() {

    pitch = 0, roll = 0, yaw = 0;

    pos_offset.x     = pos_offset.y     = pos_offset.z = 0;
    pos.x            = pos.y            = pos.z        = 0;
    pos_last.x       = pos_last.y       = pos.z        = 0;

    vel_r_raw.x      = vel_r_raw.y      = 0;
    vel_r_raw_pre.x  = vel_r_raw_pre.y  = 0;
    vel_r_raw_last.x = vel_r_raw_last.y = 0;

    vel_flow.x       = vel_flow.y       = 0;
    vel_flow_last.x  = vel_flow_last.y  = 0;

    vel.x            = vel.y            = 0;
    vel_last.x       = vel_last.y       = 0;

    pos_ef_m.x       = pos_ef.y         = pos_ef.z    = 0;
    pos_ef_m.z       = 0;

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
    float dt_s = (float)dt.get_dt_ms() / 1000.0f;                         // s

    /// 均值滤波
    double vr_x, vr_y;
    vr_x = (vel_r_raw.x + vel_r_raw_pre.x + vel_r_raw_last.x) / 3.0f;     // pixel
    vr_y = (vel_r_raw.y + vel_r_raw_pre.y + vel_r_raw_last.y) / 3.0f;

    /// 除以时间，这个是因为底层在处理包的时候乘以了时间, 单位是s
    vr_x /=  (double)dt_s;                                                // pixel/s
    vr_y /=  (double)dt_s;

    /// 摄像机像素移动结合视场角转换为速度
    /// 相似三角形算出
    double vx_gnd, vy_gnd;
    vx_gnd = (vr_x / flow_img_width)  * ccd_width  * (ground_distance * (double)1000.0f / scope_length);    // mm/s
    vy_gnd = (vr_y / flow_img_heigth) * ccd_heigth * (ground_distance * (double)1000.0f / scope_length);
    vx_gnd /= (double)1000.0f;
    vy_gnd /= (double)1000.0f;                                                                              // mm/s->m/s

    /// 速度积分变为位移
    pos_last = pos;
    pos.x += (float)vx_gnd * dt_s;                                                                          // m
    pos.y += (float)vy_gnd * dt_s;

    /// IMU得到的欧拉角角度积分变成角量，用来抵消角度运动时造成的误差
    roll_i += roll;                                                                                         // rad
    pitch_i += pitch;

    /// 角运动乘以高度转线运动
    //double x_int, y_int;
    //x_int = roll_i  * ground_distance;                                                              // m
    //y_int = pitch_i * ground_distance;


    /// 积分数据融合
    //pos_x -= x_int;                                                                                 // m
    //pos_y -= y_int;

    /// 倾斜修正
    pos.y *= (float)cos(pitch);                                                                       // m
    pos.x *= (float)cos(roll);

    /// 位移LPF
    pos.x = 0.8 * pos.x + 0.2 * pos_last.x;                                                           // m
    pos.y = 0.8 * pos.y + 0.2 * pos_last.y;
    pos.z = 0.8 * (float)ground_distance + 0.2 * pos_last.z;

    /// 微分得到速度
    vel_last = vel;                                                                                   // m/s
    vel.x = (pos.x - pos_last.x) / dt_s;
    vel.y = (pos.y - pos_last.y) / dt_s;

    /// 速度LPF
    vel.x = 0.8 * vel.x + 0.2 * vel_last.x;                                                           // m/s
    vel.y = 0.8 * vel.y + 0.2 * vel_last.y;

    /// 对机体转对地
    pos_ef.x = pos.y * (float)cos(yaw) - pos.x * (float)sin(yaw);  // north,    pos.x * (float)cos(yaw) - pos.y * (float)sin(yaw), m
    pos_ef.y = pos.y * (float)sin(yaw) + pos.x * (float)cos(yaw);  // east,     pos.x * (float)sin(yaw) + pos.y * (float)cos(yaw), m

    vel_ef.x = vel.y * (float)cos(yaw) - vel.x * (float)sin(yaw);
    vel_ef.y = vel.y * (float)cos(yaw) - vel.x * (float)sin(yaw);                                     // m/s

    ///
    /// 官方数据更新
    Vector2f vel_flow_temp;
    // LPF
    vel_flow_temp.x = 0.8 * vel_flow.x + 0.2 * vel_flow_last.x;                                       // m/s
    vel_flow_temp.y = 0.8 * vel_flow.y + 0.2 * vel_flow_last.y;
    // 积分
    pos_m.x += (float)vel_flow_temp.x * dt_s;                                                         // m
    pos_m.y += (float)vel_flow_temp.y * dt_s;                                                         // m
    // 机体坐标转ef
    pos_ef_m.x = pos_m.y * (float)cos(yaw) - pos_m.x * (float)sin(yaw);                               // north
    pos_ef_m.y = pos_m.y * (float)sin(yaw) + pos_m.x * (float)cos(yaw);                               // east
    pos_ef_m.z = (float)ground_distance;                                                              // up, m
    pos_ef_m.z = pos_ef_m.z;

    vel_ef_m.x = vel_flow_temp.y * (float)cos(yaw) - vel_flow_temp.x * (float)sin(yaw);               // m/s
    vel_ef_m.y = vel_flow_temp.y * (float)cos(yaw) - vel_flow_temp.x * (float)sin(yaw);

    //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "pos xy: %f, %f", (double)pos_ef_m.x, (double)pos_ef_m.y);
    return 0;
}

