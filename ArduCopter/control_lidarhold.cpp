#include "Copter.h"
#include <sys/time.h>

/*
 * Init and run calls for lidarhold flight mode
 */
/**
 * @brief The __lidar_ctrl class
 * 策略：
 *      因为四轴的运动线加速度是由欧拉角决定的
 *      所以在自稳环外面再套速度环和位置环，即
 *      4级的PID
 *      位置->速度->角度->角速度
 *
 *  1
 *      角度和角速度用现有的stabilize模式下的控制
 *      速度环的输入输出给get_desired_lean_angle，这样做是为了保证稳定性，也是为了很好的归一化速度
 *
 *  2
 *      速度环输出
 *          roll, pitch: -4500~4500
 *      极速状态下测出的dx, dy
 *          ±150左右
 *      这边暂时先考虑限制幅度
 *          RC的输出满是4500，这边只用1500
 *
 *      所以
 *          P大概取10
 *
 *  3
 *      因为正常都是先调内环再调外环
 *      所以外环的输出应该和内环的输入差不多
 */
#define VEL_PID_MAX 1500.0f
#define POS_PID_MAX 150.0f

class __lidar_ctrl {
public:

    __lidar_ctrl() {
        vel_ctrl = new AC_PI_2D(10.0f, 0.0f, 150.0f, 6.0f, 0.3f);
        pos_ctrl = new AC_PI_2D(0.0f,  0.0f, 150.0f, 6.0f, 0.3f);

        vx = vy = 0;
        x  = y  = 0.0f;
        roll_out  = pitch_out = 0.0f;
        t_now.tv_sec  = t_now.tv_usec  = 0;
        t_last.tv_sec = t_last.tv_usec = 0.0f;
    }

    ~__lidar_ctrl() {
        delete vel_ctrl;
        delete pos_ctrl;
    }

    double get_Roll_Output()  { return roll_out;  }

    double get_Pitch_Output() { return pitch_out; }

    double get_dt() { return dt; }

    void set_Vel(double vx_in, double vy_in) { vx = vx_in; vy = vy_in; }        // roll, pitch

    void set_Pos(double x_in,  double y_in)  { x  = x_in;  y  = y_in;  }        // roll, pitch

    void update_Time() {

        t_last = t_now;
        gettimeofday(&t_now, NULL);
        if (t_last.tv_usec > t_now.tv_usec)
            dt = t_now.tv_usec + 1e6 - t_last.tv_usec;
        else
            dt = t_now.tv_usec - t_last.tv_usec;

        // us->ms
        dt /= 1e3;

    }// int update_Time()

    void calc_Vel_Controller(double vx_tgt, double vy_tgt, double t0) {
        Vector2f in;
        Vector2f out(0.0f, 0.0f);

        in.x = vx_tgt - vx; in.y = vy_tgt - vy;
        vel_ctrl->set_dt(t0);
        vel_ctrl->set_input(in);
        out = vel_ctrl->get_pi();

        // 输出限幅
        if (out.x >= VEL_PID_MAX)
            roll_out = VEL_PID_MAX;
        else if (out.x <= -VEL_PID_MAX)
            roll_out = -VEL_PID_MAX;
        else
            roll_out = out.y;

        if (out.y >= VEL_PID_MAX)
            pitch_out  = VEL_PID_MAX;
        else if (out.y <= -VEL_PID_MAX)
            pitch_out = -VEL_PID_MAX;
        else
            pitch_out =out.y;
    }

    void calc_Pos_Controller(double t0) {

    }

private:

    AC_PI_2D* vel_ctrl;
    AC_PI_2D* pos_ctrl;

    double vx, vy;
    double x,  y;
    double roll_out, pitch_out;

    timeval t_now, t_last;
    double  dt;                 // ms

};

__lidar_ctrl ctrl;
// 以下来自usercode
extern bool is_lidar_online;
extern bool is_lidarpos_updated;
extern double lidar_x,  lidar_y;
extern double lidar_dx, lidar_dy;

// stabilize_init - initialise stabilize controller
bool Copter::lidarhold_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // 如果激光雷达不在进入这个模式没有意义
    if (!is_lidar_online) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_ERROR, "rplidar is offline, mode remain unchanged..");
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::lidarhold_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    /// debug
    //  放在这里，看它能不能算
    if (is_lidarpos_updated) {
        ctrl.update_Time();
        ctrl.set_Vel(lidar_dx, lidar_dy);
        ctrl.calc_Vel_Controller(0, 0, ctrl.get_dt() / 1e3);
        // 打印数据
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "rc_x: %f, rc_y: %f", ctrl.get_Roll_Output(), ctrl.get_Pitch_Output());
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "dt: %f", ctrl.get_dt() / 1e3);

        is_lidarpos_updated = false;
    }

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
