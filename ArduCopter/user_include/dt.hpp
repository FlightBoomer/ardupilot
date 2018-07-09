#ifndef DT_HPP
#define DT_HPP

#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

class __dt {
public:

    __dt() {
        t_now.tv_sec  = t_last.tv_sec  = 0;
        t_now.tv_usec = t_last.tv_usec = 0;
        dt_ms = 0.;
        dt_us = 0.;
        dt_s  = 0.;
    }

    void update() {
        t_last = t_now;
        gettimeofday(&t_now, NULL);

        dt_us = t_now.tv_usec - t_last.tv_usec;
        if (dt_us < 0.)
            dt_us += 1.e6;
        dt_ms = dt_us / 1.e3;

        dt_s = (t_now.tv_sec - t_last.tv_sec) +
               (t_now.tv_usec - t_last.tv_usec) / 1.e6;
    }

    double get_dt_ms() { return dt_ms; }

    double get_dt_us() { return dt_us; }

    double get_dt_s()  { return dt_s;  }

private:
    struct timeval t_now;
    struct timeval t_last;

    double dt_ms;
    double dt_us;
    double dt_s;

};

#endif // DT_HPP
