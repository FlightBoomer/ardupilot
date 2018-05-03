#include "Copter.h"

#include "include/dt.hpp"
#include "../libraries/AC_AttitudeControl/AC_PosControl.h"
#include "../libraries/AC_PID/AC_P.h"
#include "../libraries/AC_PID/AC_PI_2D.h"

/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Copter::ofloiter_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Alt Hold if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    // initialize vertical speeds and leash lengths
    of_pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    of_pos_control->set_accel_z(g.pilot_accel_z);

    // 初始化位置控制
    of_pos_control->init_xy_controller(true);

    // initialise position and desired velocity
    if (!of_pos_control->is_active_z()) {
        of_pos_control->set_alt_target_to_current_alt();
        of_pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    // stop takeoff if running
    takeoff_stop();

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Copter::ofloiter_run()
{
    AltHoldModeState althold_state;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    of_pos_control->set_speed_z(-g.pilot_velocity_z_max / 2, g.pilot_velocity_z_max / 2);
    of_pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    //get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control->get_althold_lean_angle_max());

    Vector2f vel_xy = flow_ctrl.get_Velocity_m();
    vel_xy.x *= 100.0f;
    vel_xy.y *= 100.0f;
    static int j = 0;
    j++;
    if (j >= 40) {
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "control optflow vel xy: %f, %f", (double)vel_xy.y, (double)vel_xy.x);
        j = 0;
    }
    of_xy->set_current_vel_body_xy(vel_xy.y, vel_xy.x);
    of_xy->update_vel_controller_body_xy(ekfNavVelGainScaler);
    target_pitch = of_xy->get_pitch();
    target_roll  = of_xy->get_roll();

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max / 2, g.pilot_velocity_z_max / 2);

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Alt Hold State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        of_pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        of_pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        of_pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
        }

        // get take-off adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call position controller
        of_pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        of_pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        of_pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        of_pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        of_pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        avoid.adjust_roll_pitch(target_roll, target_pitch, aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, of_pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        of_pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        of_pos_control->update_z_controller();
        break;
    }

    ///
    static int i = 0;
    i++;
    if (i % 40 == 0) {
        float alt_err = of_pos_control->get_alt_error();
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "alt_error: %f", (double)alt_err);
    }
}
