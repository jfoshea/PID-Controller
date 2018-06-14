#ifndef PID_H
#define PID_H

#include <vector>

typedef enum 
{
  Kp_PARAM = 0,
  Ki_PARAM,
  Kd_PARAM,
  PARAM_LAST
} PID_PARAM_S;

class PID {
public:
  // Errors
  double p_error;
  double i_error;
  double d_error;

  // Coefficients
  double Kp;
  double Ki;
  double Kd;

  double dpp;
  double dpi;
  double dpd;

  int eval_count;
  const int num_evals = 200;  

  PID();

  virtual ~PID();

  void Init(double Kp, double Ki, double Kd);

  void UpdateError(double cte);

  double TotalError();
  
  void FindCoefficients( const double total_error );
};

#endif /* PID_H */
