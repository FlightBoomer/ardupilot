#include "Copter.h"

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool Copter::ofloiter_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Loiter if the Rotor Runup is not complete
    if (!ignore_checks && !motors->rotor_runup_complete()){
        return false;
    }
#endif

    if (position_ok() || ignore_checks) {

        // set target to current position
        of_wp_nav->init_loiter_target();

        // initialize vertical speed and acceleration
        of_pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        of_pos_control->set_accel_z(g.pilot_accel_z);

        // initialise position and desired velocity
        if (!of_pos_control->is_active_z()) {
            of_pos_control->set_alt_target_to_current_alt();
            of_pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
        }

        return true;
    }else{
        return false;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void Copter::ofloiter_run()
{
    LoiterModeState loiter_state;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speed and acceleration
    of_pos_control->set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    of_pos_control->set_accel_z(g.pilot_accel_z);

    of_wp_nav->set_rangefinder_alt(true, true, flow_ctrl.get_GroundDistance());


    // process pilot inputs unless we are in radio failsafe
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // process pilot's roll and pitch input
        of_wp_nav->set_pilot_desired_acceleration(channel_roll->get_control_in(), channel_pitch->get_control_in());

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        of_wp_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        of_wp_nav->loiter_soften_for_landing();
    }

#if FRAME_CONFIG == HELI_FRAME
    // helicopters are held on the ground until rotor speed runup has finished
    bool takeoff_triggered = (ap.land_complete && (target_climb_rate > 0.0f) && motors->rotor_runup_complete());
#else
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);
#endif

    // Loiter State Machine Determination
    if (!motors->armed() || !motors->get_interlock()) {
        loiter_state = Loiter_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
        loiter_state = Loiter_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        loiter_state = Loiter_Landed;
    } else {
        loiter_state = Loiter_Flying;
    }

    // Loiter State Machine
    switch (loiter_state) {

    case Loiter_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
#if FRAME_CONFIG == HELI_FRAME
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
#else
        of_wp_nav->init_loiter_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        of_pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        of_wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(of_wp_nav->get_roll(), of_wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());
        of_pos_control->update_z_controller();
        break;

    case Loiter_Takeoff:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            //set_throttle_takeoff();
            of_pos_control->init_takeoff();
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // run loiter controller
        of_wp_nav->update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(of_wp_nav->get_roll(), of_wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());

        // update altitude target and call position controller
        of_pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        of_pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        of_pos_control->update_z_controller();
        break;

    case Loiter_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        of_wp_nav->init_loiter_target();
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        of_pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        of_pos_control->update_z_controller();
        break;

    case Loiter_Flying:

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        if (do_precision_loiter()) {
            precision_loiter_xy();
        }
#endif

        // run loiter controller
        double of_speed_limit =  ekfGndSpdLimit / 2;
        of_wp_nav->update_loiter(of_speed_limit, ekfNavVelGainScaler);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(of_wp_nav->get_roll(), of_wp_nav->get_pitch(), target_yaw_rate, get_smoothing_gain());

        // adjust climb rate using rangefinder
        if (rangefinder_alt_ok()) {
            // if rangefinder is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, of_pos_control->get_alt_target(), G_Dt);
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update altitude target and call position controller
        of_pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        of_pos_control->update_z_controller();
        break;
    }
}

