#include "optflow_ctrl.h"

void __optflow_inav_intf::update_BasicData(AP_InertialNav_NavEKF in) {
    _relpos_cm   = in.get_position();         // NEU
    _velocity_cm = in.get_velocity();         // NEU
    _pos_z_rate  = in.get_pos_z_derivative();

    /// 这两部分传不过来,用自己的AHRS读
    //_haveabspos  = _ahrs_ekf.get_position(_abspos);
    //_ahrs_ekf    = in._ahrs_ekf;
}

void __optflow_inav_intf::update_Flow(__optflow_ctrl in, bool is_use_flow_m) {
    float x, y, z;
    Vector2f _vec2f;

    dt.update();
    float dt_s = (float)dt.get_dt_ms() / 1000.0f;                                                 // s

    if (!is_use_flow_m) {
        //
        _relpos_cm = in.get_Position_ef();
        _relpos_cm.x *= 100.0f, _relpos_cm.y *= 100.0f, _relpos_cm.z *= 100.0f;            // m->cm

        //
        _vec2f = in.get_Velocity_ef();                                                     // m/s
        x = _vec2f.x * 100.0f, y = _vec2f.y * 100.0f;                                      // m/s->cm/s
        ground_dst_last = ground_dst;
        ground_dst = in.get_GroundDistance();                                              // m
        z = (float)(ground_dst - ground_dst_last) / dt_s;                                  // m/s
        z *= 100.0f;                                                                       // m/s->cm/s

        _velocity_cm.x = x;                                                                // cm/s
        _velocity_cm.y = y;                                                                // cm/s
        _velocity_cm.z = z;                                                                // cm/s
        _pos_z_rate    = z;                                                                // cm/s

        _abspos.alt = (int32_t)_relpos_cm.z;       ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
        _abspos.lat = (int32_t)_relpos_cm.x;       ///< param 3 - Latitude * 10**7
        _abspos.lng = (int32_t)_relpos_cm.y;       ///< param 4 - Longitude * 10**7

        _haveabspos = true;
    }
    else {
        _relpos_cm = in.get_Position_ef_m();                                              // m
        _relpos_cm.x *= 100.0f;                                                           // m->cm
        _relpos_cm.y *= 100.0f;
        _relpos_cm.z *= 100.0f;

        _vec2f = in.get_Velocity_ef_m() * 100.0f;                                         // m/s
        ground_dst_last = ground_dst;
        ground_dst = in.get_GroundDistance();                                             // m
        z = (float)(ground_dst - ground_dst_last) / dt_s;                                 // m/s
        _velocity_cm.x = _vec2f.x * 100.0f;                                               // m/s->cm/s
        _velocity_cm.y = _vec2f.y * 100.0f;
        _velocity_cm.z = z * 100.0f;                                                      // m/s->cm/s
        _pos_z_rate    = _velocity_cm.z;

        _abspos.alt = (int32_t)_relpos_cm.z;       ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
        _abspos.lat = (int32_t)_relpos_cm.x;       ///< param 3 - Latitude * 10**7
        _abspos.lng = (int32_t)_relpos_cm.y;       ///< param 4 - Longitude * 10**7

        _haveabspos = true;

        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "pos xy: %f, %f", (double)_relpos_cm.x, (double)_relpos_cm.y);
        //GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "_relpos_cm.z: %f", (double)_relpos_cm.z);
        GCS_MAVLINK::send_statustext_all(MAV_SEVERITY_INFO, "gnd_dist_rate: %f", (double)_velocity_cm.z);
    }
}
