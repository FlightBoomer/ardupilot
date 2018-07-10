#ifndef CONTROL_VISUAL_LOITER_HPP
#define CONTROL_VISUAL_LOITER_HPP

#include <unistd.h>

#include "../Copter.h"
#include "../../libraries/AC_PID/AC_PID.h"

#include "user_config.h"
#include "user_filter.hpp"

// https://blog.csdn.net/HAHAandHEHE/article/details/79617028

#define IIR_ORDER  4      //使用IIR滤波器的阶数

class __visual_loiter_ctrl {
public:
    __visual_loiter_ctrl() :
    roll_pos_ctrl(0.,  0., 0., 0.05, 20., 2e-1),
    pitch_pos_ctrl(0., 0., 0., 0.05, 20., 2e-1),
    roll_vel_ctrl(0.,  0., 0., 150, 20., 2e-1),
    pitch_vel_ctrl(0., 0., 0., 150, 20., 2e-1) {

        memset(InPut_IIR,  0, sizeof(double) * 2 * (IIR_ORDER + 1));
        memset(OutPut_IIR, 0, sizeof(double) * 2 * (IIR_ORDER + 1));
    }

    void set_Pos(__vec3f pos_in) {
        pos_last = pos;
        pos      = pos_in;
    }

    void set_Takeoff_Pos(__vec3f pos_in) {
        pos_takeoff = pos_in;
    }

    void set_Vel(__vec3f vel) {
        pos_rate = vel;
    }

    int update() {

        double pos_rate_x_temp, pos_rate_y_temp;

        /// do LPF for Pos
        pos.x = LPF_I(pos.x, pos_last.x, 0.612f);
        pos.y = LPF_I(pos.y, pos_last.y, 0.612f);

        /// do IIR for Vel
        pos_rate_x_temp = pos_rate.x;
        pos_rate_y_temp = pos_rate.y;
        pos_rate.x = IIR_I_Filter(pos_rate_x_temp, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
        pos_rate.y = IIR_I_Filter(pos_rate_y_temp, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);

        /// RC to Expectations
        ctrl_Exp_2_Pos();

        /// Shell PID
        ctrl_Pos_2_Vel();

        /// Core PID
        ctrl_Vel_2_LeanAngle();


        return 0;
    }

    int update(int16_t roll_rc, int16_t pitch_rc, __vec3f pos_in, __vec3f vel_in) {

        set_Roll_Exp(roll_rc);
        set_Pitch_Exp(pitch_rc);
        set_Pos(pos_in);
        set_Vel(vel_in);

        return update();

    }

    void set_Roll_Exp(int16_t in)  { roll_exp_in  = in; }
    void set_Pitch_Exp(int16_t in) { pitch_exp_in = in; }

    int16_t roll_desired_lean_angle()  { return roll_angle_out; }
    int16_t pitch_desired_lean_angle() { return pitch_angle_out; }

protected:

    const double sensitivity_rc_2_pos = 20.0f;
    const double pos_exp_3dB = 5.;      // need calibrating
    const double vel_limit   = 0.75f;   // need calibrating


    __vec3f pos, pos_last, pos_takeoff;
    __vec3f pos_rate;

    // IIR filter parameters
    double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
    double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
    double InPut_IIR[2][IIR_ORDER+1];
    double OutPut_IIR[2][IIR_ORDER+1];

    AC_PID roll_pos_ctrl, pitch_pos_ctrl;
    AC_PID roll_vel_ctrl, pitch_vel_ctrl;

    int16_t roll_exp_in,    pitch_exp_in;       // input data suits for RC sticks best
    int16_t roll_angle_out, pitch_angle_out;    // output data for attitude control

    double _roll_pos_desired, _pitch_pos_desired;
    double _pos_err_roll, _pos_err_pitch;
    double _pos_out_roll, _pos_out_pitch;       // shell PID(pos)   output
    double _vel_err_roll, _vel_err_pitch;       // core  PID(speed) input

    void ctrl_Exp_2_Pos() {
        _roll_pos_desired  = (double)roll_exp_in  / sensitivity_rc_2_pos;
        _pitch_pos_desired = (double)pitch_exp_in / sensitivity_rc_2_pos;
    }

    void ctrl_Pos_2_Vel() {
        /// Calculate Error
        //_pos_err_roll  = _roll_pos_desired  - pos.y;
        //_pos_err_pitch = _pitch_pos_desired - pos.x;
        _pos_err_roll  = _roll_pos_desired;
        _pos_err_pitch = _pitch_pos_desired;

        // use a square err to make control smoother
        if (fabs(_pos_err_roll) > pos_exp_3dB) {
            if (_pos_err_roll > 0) {
                if (_pos_err_roll > 2 * pos_exp_3dB)
                    _pos_err_roll = 2 * pos_exp_3dB;
                else    // pos_exp_3dB < _pos_err_roll < 2 * _pos_err_roll
                    _pos_err_roll = pos_exp_3dB + sqrt(_pos_err_roll - pos_exp_3dB);
            }
            else {  // _pos_err_roll < 0
                if (_pos_err_roll < -2 * pos_exp_3dB)
                    _pos_err_roll = -2 * pos_exp_3dB;
                else    // -2 * pos_exp_3dB < _pos_err_roll < _pos_err_roll
                    _pos_err_roll = -pos_exp_3dB - sqrt(-_pos_err_roll - pos_exp_3dB);

            }
        }

        if (fabs(_pos_err_pitch) > pos_exp_3dB) {
            if (_pos_err_pitch > 0) {
                if (_pos_err_pitch > 2 * pos_exp_3dB)
                    _pos_err_pitch = 2 * pos_exp_3dB;
                else    // pos_exp_3dB < _pos_err_roll < 2 * _pos_err_roll
                    _pos_err_pitch = pos_exp_3dB + sqrt(_pos_err_pitch - pos_exp_3dB);
            }
            else {  // _pos_err_roll < 0
                if (_pos_err_pitch < -2 * pos_exp_3dB)
                    _pos_err_pitch = -2 * pos_exp_3dB;
                else    // -2 * pos_exp_3dB < _pos_err_roll < _pos_err_roll
                    _pos_err_pitch = -pos_exp_3dB - sqrt(-_pos_err_pitch - pos_exp_3dB);

            }
        }

        /// Update PID Controller
        roll_pos_ctrl.set_input_filter_all(_pos_err_roll);
        pitch_pos_ctrl.set_input_filter_all(_pos_err_pitch);

        _pos_out_roll  = roll_pos_ctrl.get_pid();
        _pos_out_pitch = pitch_pos_ctrl.get_pid();
    }

    void ctrl_Vel_2_LeanAngle() {
        /// Calculate Errors
        _vel_err_roll  = _pos_out_roll  - pos_rate.y;
        _vel_err_pitch = _pos_out_pitch - pos_rate.x;

        // limit max velocity
        if (_vel_err_roll > vel_limit)
            _vel_err_roll = vel_limit;
        else if (_vel_err_roll < -vel_limit)
            _vel_err_roll = -vel_limit;

        if (_vel_err_roll > vel_limit)
            _vel_err_roll = vel_limit;
        else if (_vel_err_roll < -vel_limit)
            _vel_err_roll = -vel_limit;

        /// Update PID_Controller
        roll_vel_ctrl.set_input_filter_all(_vel_err_roll);
        pitch_vel_ctrl.set_input_filter_all(_vel_err_pitch);

        roll_angle_out  = (int16_t)roll_vel_ctrl.get_pid();
        pitch_angle_out = (int16_t)pitch_vel_ctrl.get_pid();
    }
};

// Velocity only for better debugging
class __visual_loiter_ctrl_vel : public __visual_loiter_ctrl {
public:
    __visual_loiter_ctrl_vel() : __visual_loiter_ctrl() {

    }

    int update_VelCtrl_RC() {

        double pos_rate_x_temp, pos_rate_y_temp;

        /// do LPF for Pos
//      pos.x = LPF_I(pos.x, pos_last.x, 0.612f);
//      pos.y = LPF_I(pos.y, pos_last.y, 0.612f);

        /// do IIR for Vel
        pos_rate_x_temp = pos_rate.x;
        pos_rate_y_temp = pos_rate.y;
        pos_rate.x = IIR_I_Filter(pos_rate_x_temp, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
        pos_rate.y = IIR_I_Filter(pos_rate_y_temp, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);

        /// directly connect input to core PID
        ctrl_Exp_2_Vel();
        _pos_out_roll  = _roll_vel_exp;
        _pos_out_pitch = _pitch_vel_exp;

        ctrl_Vel_2_LeanAngle();

        return 0;
    }

    int update_VelCtrl_RC(int16_t roll_rc, int16_t pitch_rc, __vec3f vel_in) {

        set_Roll_Exp(roll_rc);
        set_Pitch_Exp(pitch_rc);
        set_Vel(vel_in);

        return update_VelCtrl_RC();

    }

    void set_Roll_Vel_Exp(int16_t in)  { roll_vel_exp_in  = in; }

    void set_Pitch_Vel_Exp(int16_t in) { pitch_vel_exp_in = in; }

protected:

    const double vel_rc_sensivity = 250.0f;     // need calibrating

    int16_t roll_vel_exp_in, pitch_vel_exp_in;
    double  _roll_vel_exp, _pitch_vel_exp;

    void ctrl_Exp_2_Vel() {
        _roll_vel_exp  = (double)roll_vel_exp_in / vel_rc_sensivity;
        _pitch_vel_exp = (double)roll_vel_exp_in / vel_rc_sensivity;
    }

};

// Input from real velocity exptectations
class __visual_loiter_ctrl_auto : public __visual_loiter_ctrl {
    __visual_loiter_ctrl_auto() : __visual_loiter_ctrl() {

    }

    int update_Auto() {
        double pos_rate_x_temp, pos_rate_y_temp;

        /// do LPF for Pos
        pos.x = LPF_I(pos.x, pos_last.x, 0.612f);
        pos.y = LPF_I(pos.y, pos_last.y, 0.612f);

        /// do IIR for Vel
        pos_rate_x_temp = pos_rate.x;
        pos_rate_y_temp = pos_rate.y;
        pos_rate.x = IIR_I_Filter(pos_rate_x_temp, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
        pos_rate.y = IIR_I_Filter(pos_rate_y_temp, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);

        /// RC to Expectations
        //ctrl_Exp_2_Pos();

        /// Shell PID
        ctrl_Pos_2_Vel();

        /// Core PID
        ctrl_Vel_2_LeanAngle();


        return 0;
    }

    int update_Auto(int16_t roll_rc, int16_t pitch_rc, __vec3f pos_in, __vec3f vel_in) {

        set_Roll_Exp(roll_rc);
        set_Pitch_Exp(pitch_rc);
        set_Pos(pos_in);
        set_Vel(vel_in);

        return update_Auto();

    }

    void set_Pos_Pitch_Tgt(double in) {
        _pitch_pos_desired = in;
    }

    void set_Pos_Roll_Tgt(double in) {
        _roll_pos_desired = in;
    }

protected:

};


#endif // CONTROL_VISUAL_LOITER_HPP
