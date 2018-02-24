#include "Copter.h"
#include <sys/time.h>

/*
 * Init and run calls for lidarhold flight mode
 */
class __lidar_ctrller {
public:

    __lidar_ctrller() {

    }

    int init() {

    }

    double get_Roll_Output()  { return roll_out;  }

    double get_Pitch_Output() { return pitch_out; }

    double get_dt() { return dt; }

    void set_Vel(double vx_in, double vy_in) { vx = vx_in; vy = vy_in; }

    void set_Pos(double x_in, double y_in)   { x  = x_in;  y  = y_in;  }

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

    void calc_Vel_Controller() {

    }

    void calc_Pos_Controller() {

    }

private:

    AC_PID_2D vel_ctrl;
    AC_PID_2D pos_ctrl;

    double vx, vy;
    double x,  y;
    double roll_out, pitch_out;

    timeval t_now, t_last;
    double  dt;

};

// stabilize_init - initialise stabilize controller
bool Copter::lidarhold_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
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
