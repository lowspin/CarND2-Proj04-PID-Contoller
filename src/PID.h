#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * * for Twiddle
  */
  double avg_cte;
  double best_avgcte;
  bool still_twiddling;
  double framecount;
  double Params[3];
  double dP[3];
  int tuning_param;
  bool stage2;
  int status;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_, bool still_twiddling_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
  * Tune the next parameter for Twiddle.
  */
  void TuneNext();
};

#endif /* PID_H */
