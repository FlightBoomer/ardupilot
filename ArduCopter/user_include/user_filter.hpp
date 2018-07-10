#ifndef USER_FILTER_HPP
#define USER_FILTER_HPP

#include <iostream>
#include <cmath>

// IIR Filter
// y(n) = b0*x(n) + b1*x(n-1) + b2*x(n-2) -
//        a1*y(n-1) - a2*y(n-2
inline double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na)
{
  double z1,z2;
  short i;
  double OutData;

  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }

  x[0] = InData;

  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }

  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }

  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }

  y[0] = z1 - z2;
  OutData = y[0];

  return OutData;
}

// Low-Pass Filter
inline double LPF_I(double in, double in_last, double k1) {

    // 0 < k1 < 1 or there will be something wrong
    return in * k1 + in_last * (1 - k1);
}

#endif // USER_FILTER_HPP
