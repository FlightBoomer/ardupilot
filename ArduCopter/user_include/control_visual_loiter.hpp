#ifndef CONTROL_VISUAL_LOITER_HPP
#define CONTROL_VISUAL_LOITER_HPP

#include <unistd.h>

#include "../Copter.h"
#include "../../libraries/AC_PID/AC_PID.h"

#include "user_config.h"

// https://blog.csdn.net/HAHAandHEHE/article/details/79617028

class __visual_loiter_ctrl {
public:
    __visual_loiter_ctrl() :
    roll_pos_ctrl(0.,  0., 0., 0.05, 20., 2e-1),
    pitch_pos_ctrl(0., 0., 0., 0.05, 20., 2e-1),
    roll_vel_ctrl(0.,  0., 0., 150, 20., 2e-1),
    pitch_vel_ctrl(0., 0., 0., 150, 20., 2e-1) {

    }

    void set_Pos(__vec3f pos_in) {
        pos_last = pos;
        pos      = pos_in;
    }

    void set_Takeoff_Pos(__vec3f pos_in) {
        pos_takeoff = pos_in;
    }

    int update() {
        return 0;
    }

    void set_Roll_Exp(int16_t in)  { roll_exp_in  = in; }
    void set_Pitch_Exp(int16_t in) { pitch_exp_in = in; }

    int roll_desired_lean_angle()  { return roll_angle_out; }
    int pitch_desired_lean_angle() { return pitch_angle_out; }

protected:

    __vec3f pos, pos_last, pos_takeoff;
    __vec3f pos_rate;

    AC_PID roll_pos_ctrl, pitch_pos_ctrl;
    AC_PID roll_vel_ctrl, pitch_vel_ctrl;

    int16_t roll_exp_in,    pitch_exp_in;
    int16_t roll_angle_out, pitch_angle_out;



};

#endif // CONTROL_VISUAL_LOITER_HPP
