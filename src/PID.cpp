#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  this->d_error = cte - this->p_error;
  this->i_error = this->d_error + cte;
  this->p_error = cte;
}

double PID::TotalError() {
}

