#ifndef DT_HPP
#define DT_HPP

#include <sys/time.h>

class __dt {
public:

    __dt() {

    }// __dt()

    double get() {

        update_Time();

        return dt;

    }// double get()

private:

    timeval t_now, t_last;
    double  dt;                 // ms

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

};

#endif // DT_HPP
