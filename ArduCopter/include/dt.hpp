#ifndef DT_HPP
#define DT_HPP

class __dt {
public:

    __dt() {
        t = t_last = 0;
        dt = 0;
        t_elapsed = 0;
    }

    double get_dt_ms() { return dt / (double)1000.0f; }
    double get_t_all_ms() { return t_elapsed / (double)1000.0f; }

    double update() {

        t_last = t;
        t = AP_HAL::micros();
        dt = (double)(t - t_last);

        return dt / (double)1000.0f;
    }

private:

    uint32_t t, t_last;
    double dt;                  // ms
    double t_elapsed;           // ms

};

#endif // DT_HPP
