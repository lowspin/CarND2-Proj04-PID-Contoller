#include "PID.h"
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_, bool still_twiddling_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  d_error = 0.0;
  i_error = 0.0;
  p_error = 0.0;

  still_twiddling = still_twiddling_;
  avg_cte = 0.0;
  framecount = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  i_error = d_error + cte;
  p_error = cte;

  if (framecount>0)
    avg_cte = (avg_cte*((framecount-1)/framecount)) + (abs(cte)/framecount);
    // Check: sum_n/n = sum_n-1 + val_n / n = sum_n-1/n + val_n/n = (sum_n-1/n-1)*(n-1)/n + val_n/n = avg*(n-1)/n + val_n/n

    // use worse cte
    //avg_cte = (abs(cte)>avg_cte)?abs(cte):avg_cte;
  else
    avg_cte = abs(cte);
}

double PID::TotalError() {
  // return (-Kp*p_error); // P
  // return (-Kp*p_error -Kd*d_error); // P+D
  return (-Kp*p_error -Kd*d_error -Ki*i_error); // P+I+D
}

void PID::TuneNext() {
  tuning_param = (tuning_param+1)%3;
}
