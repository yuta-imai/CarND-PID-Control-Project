#include "PID.h"
#include <math.h> 
#include <iostream>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  cte_sum = 0.0;

  counter = 0;
  tw_best_avg_cte = -1.0;

}

double PID::NextSteerValue(double cte) {

  double steer_value = - Kp * p_error
                       - Kd * d_error
                       - Ki * i_error;

  if(steer_value > 0.3) {
    steer_value = 0.3;
  } else if (steer_value < -0.3) {
    steer_value = -0.3;
  }

  std::cout << "Total Error: " << TotalError() << std::endl;
  std::cout << "Step Counter: " << counter << std::endl;
  std::cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << std::endl;
  return steer_value;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  cte_sum += pow(cte,2);
  counter++;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return cte_sum/counter;  // TODO: Add your total error calc here!
}

void PID::InitTwiddle(double twiddle_p, double twiddle_i, double twiddle_d,  double twiddle_tolerance, unsigned long long twiddle_after) {

  twiddeling = true;
  this->twiddle_tolerance = twiddle_tolerance;
  this->twiddle_after = twiddle_after;
  tw_param_diffs[0] = twiddle_p;
  tw_param_diffs[1] = twiddle_d;
  tw_param_diffs[2] = twiddle_i;
  tw_best_avg_cte = -1;

}

void PID::ResetTotalError() {
	counter = 0;
	cte_sum = 0;
}

void PID::Twiddle() {
  if(twiddeling && (counter > twiddle_after)) {
    double current_avg_cte = TotalError();
    bool go_to_next = false;

    if(tw_best_avg_cte < 0) {
      tw_best_avg_cte = current_avg_cte;
      tw_curr_direction = 0;
      tw_curr_param = 0;

      while(tw_param_diffs[tw_curr_param] == 0) {
        tw_curr_param = (tw_curr_param + 1) % 3;
      }
    }

    double sum_diff = tw_param_diffs[0] + tw_param_diffs[1] + tw_param_diffs[2] ;
    std::cout << "Tolerance: " << sum_diff << std::endl;
    if(sum_diff > twiddle_tolerance) {
      double increment_param = 0.0;

      switch(tw_curr_direction) {
        case 0:
          increment_param= tw_param_diffs[tw_curr_param];
          tw_curr_direction = 1;
          break;
        case 1:
          if(current_avg_cte < tw_best_avg_cte) {
            tw_best_avg_cte = current_avg_cte;
            tw_param_diffs[tw_curr_param] *= 1.1;
            go_to_next = true;
          } else {
            increment_param = -2 * tw_param_diffs[tw_curr_param];
            tw_curr_direction = -1;
          }
          break;
        case -1:
          if(current_avg_cte < tw_best_avg_cte) {
            tw_best_avg_cte = current_avg_cte;
            tw_param_diffs[tw_curr_param] *= 1.1;
          } else {
            increment_param = tw_param_diffs[tw_curr_param];
  					tw_param_diffs[tw_curr_param] *= 0.9;
          }
          go_to_next = true;
          break;
      }

      switch (tw_curr_param) 
      {
        case 0:
          Kp += increment_param;
          break;
        case 1:
          Kd += increment_param;
        case 2:
          Ki += increment_param;
      }

      if(go_to_next) {
        tw_curr_direction = 0;
        do {
          tw_curr_param = (tw_curr_param +1) %3;
        } while (tw_param_diffs[tw_curr_param] == 0);
      }
      ResetTotalError();
    }
  } else {
    std::cout << "Twiddling is OFF" << std::endl;;
  }
}