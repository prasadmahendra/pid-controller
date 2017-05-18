#ifndef PID_H
#define PID_H

#include <vector>

//#define DEBUG

class PID {

private:
  bool initialized;
  
  double last_cte;
  std::vector<double> steer_history;
  double sum_cte;
  
  // Errors
  double p_error;
  double i_error;
  double d_error;
  double best_error;
  
  // Coefficients
  double Kp;
  double Ki;
  double Kd;
  
  // Twiddle (coordinate ascent) coefficient deltas
  double dKp;
  double dKi;
  double dKd;
  
  int current_twiddle_param = 0;
  int steering_avg_over = 3;
  
  enum TWIDDLE_STATES {
    TWIDDLE_UP,
    TWIDDLE_UP_CHECK_ERROR,
    TWIDDLE_AFTER_RESTORE,
    TWIDDLE_RESTORE_CHECK_ERROR
  };
  
  TWIDDLE_STATES twiddle_state;
  
public:
  
  
  

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  double SteeringAngle(double min=-1, double max=1);
  
  void TwiddleCoeffsPre(double tolerance=0.2);
  void TwiddleCoeffsPost(double tolerance=0.2);
};

#endif /* PID_H */
