#include "PID.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <numeric>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  this->dKp = 1;
  this->dKi = 1;
  this->dKd = 1;
  
  this->last_cte = 0.0;
  this->sum_cte = 0.0;
  this->best_error = std::numeric_limits<double>::max();
  this->current_twiddle_param = 0;
  
  this->twiddle_state = TWIDDLE_UP;
  this->steer_history = std::vector<double>(steering_avg_over);
  
  std::cout.precision(17);
}

void PID::UpdateError(double cte) {
  if (!initialized) {
    last_cte = cte;
    initialized = true;
  }
  
  TwiddleCoeffsPre();
  
  sum_cte += cte;
 
  double diff_cte = cte - last_cte;
  p_error = cte;
  i_error = sum_cte;
  d_error = diff_cte;

  last_cte = cte;
  
  TwiddleCoeffsPost();
}

double PID::TotalError() {
  return p_error + i_error + d_error;
}

double PID::SteeringAngle(double min, double max) {
  double steer =  -(Kp * p_error + Ki * i_error + Kd * d_error);
  if( steer < min ) {
    std::cout << " steer less than min!!! " << steer << std::endl;
    steer = min;
  } else if (steer > max) {
    std::cout << " steer greater than max!!! " << steer << std::endl;
    steer = max;
  }
  
  steer_history.push_back(steer);
  if( steer_history.size() > steering_avg_over ) {
    steer_history.erase(steer_history.begin());
  }
  
  return steer;
}

void PID::TwiddleCoeffsPre(double tolerance) {
  TWIDDLE_STATES next_state = twiddle_state;
  std::vector<double> K = {this->Kp, this->Ki, this->Kd};
  std::vector<double> dK = {this->dKp, this->dKi, this->dKd};

  if( std::accumulate(dK.begin(), dK.end(), 0.0) < tolerance ) {
    std::cout << " Kp = " << std::fixed << this->Kp << " Ki = " << std::fixed << this->Ki << " Kd = " << std::fixed << this->Kd << std::fixed << " tol = " << std::accumulate(dK.begin(), dK.end(), 0.0) << std::endl;
    return;
  } else {
    std::cout << " **** dKp = " << std::fixed << this->dKp << " dKi = " << std::fixed << this->dKi << " dKd = " << std::fixed << this->dKd << " tol = " << std::accumulate(dK.begin(), dK.end(), 0.0)  << std::endl;
  }
  
  switch(twiddle_state) {
    case TWIDDLE_UP:
      K[current_twiddle_param] += dK[current_twiddle_param];
      next_state = TWIDDLE_UP_CHECK_ERROR;
#ifdef DEBUG
      std::cout << " twiddle state TWIDDLE_UP -> TWIDDLE_UP_CHECK_ERROR (" << current_twiddle_param << ")" << std::endl;
#endif
      break;

    case TWIDDLE_AFTER_RESTORE:
      next_state = TWIDDLE_RESTORE_CHECK_ERROR;
#ifdef DEBUG
      std::cout << " twiddle state TWIDDLE_AFTER_RESTORE -> TWIDDLE_RESTORE_CHECK_ERROR (" << current_twiddle_param << ")" << std::endl;
#endif
      break;
      
    default:
      std::cerr << "TwiddleCoeffsPre state not handled - " << twiddle_state << std::endl;
      break;
  }
  
  twiddle_state = next_state;
  
  this->Kp = K[0];
  this->Ki = K[1];
  this->Kd = K[2];
  this->dKp = dK[0];
  this->dKi = dK[1];
  this->dKd = dK[2];
}

void PID::TwiddleCoeffsPost(double tolerance) {
  TWIDDLE_STATES next_state = twiddle_state;
  std::vector<double> K = {this->Kp, this->Ki, this->Kd};
  std::vector<double> dK = {this->dKp, this->dKi, this->dKd};

  if( std::accumulate(dK.begin(), dK.end(), 0.0) <= tolerance ) {
    return;
  }
  
  switch(twiddle_state) {
    case TWIDDLE_UP_CHECK_ERROR:
      if ( TotalError() < best_error ) {
        best_error = TotalError();
        dK[current_twiddle_param] *= 1.1;
#ifdef DEBUG
        std::cout << " twiddle state TWIDDLE_UP_CHECK_ERROR -> TWIDDLE_UP (" << current_twiddle_param << ")" << std::endl;
#endif
        next_state = TWIDDLE_UP;
      } else {
        // restore ...
        K[current_twiddle_param] -= 2.0 * dK[current_twiddle_param];
#ifdef DEBUG
        std::cout << " twiddle state TWIDDLE_UP_CHECK_ERROR -> TWIDDLE_AFTER_RESTORE (" << current_twiddle_param << ")" << std::endl;
#endif
        next_state = TWIDDLE_AFTER_RESTORE;
      }
      break;
    case TWIDDLE_RESTORE_CHECK_ERROR:
      if ( TotalError() < best_error ) {
        best_error = TotalError();
        dK[current_twiddle_param] *= 1.1;
#ifdef DEBUG
        std::cout << " twiddle state TWIDDLE_RESTORE_CHECK_ERROR -> TWIDDLE_UP (" << current_twiddle_param << ")" << std::endl;
#endif
        next_state = TWIDDLE_UP;
      } else {
        K[current_twiddle_param] += dK[current_twiddle_param];
        dK[current_twiddle_param] *= 0.9;
#ifdef DEBUG
        std::cout << " twiddle state TWIDDLE_RESTORE_CHECK_ERROR -> TWIDDLE_UP (" << current_twiddle_param << ")" << std::endl;
#endif
        next_state = TWIDDLE_UP;
      }
      break;
    default:
      std::cerr << "TwiddleCoeffsPost state not handled - " << twiddle_state << std::endl;
      break;
  }
  
  if( next_state == TWIDDLE_UP ) {
    current_twiddle_param = (current_twiddle_param + 1) % K.size();
  }
  
  twiddle_state = next_state;
  
  this->Kp = K[0];
  this->Ki = K[1];
  this->Kd = K[2];
  this->dKp = dK[0];
  this->dKi = dK[1];
  this->dKd = dK[2];
}
