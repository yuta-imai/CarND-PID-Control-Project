#define _USE_MATH_DEFINES
#include "PID.h"
#include <math.h> 
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Kd, double Ki) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	d_error = 0.0;
	p_error = 0.0;
	i_error = 0.0;
	cte_sum = 0.0;
	step_counter = 0;
	tw_best_avg_cte = -1.0; // i.e., undefined
}

void PID::InitTwiddle(double twiddle_p, double twiddle_d, double twiddle_i, double twiddle_tolerance, unsigned long long twiddle_interval) {

	this->twiddle_tolerance = twiddle_tolerance;
	this->twiddle_interval = twiddle_interval;
	tw_param_diffs[0] = twiddle_p;
	tw_param_diffs[1] = twiddle_d;
	tw_param_diffs[2] = twiddle_i;
	tw_best_avg_cte = -1;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	cte_sum += pow(cte, 2);
	step_counter++;
}

double PID::TotalError() {
	return cte_sum / step_counter;
}

void PID::ResetTotalError() {
	step_counter = 0;
	cte_sum = 0;
}

double PID::NextSteeringValue() {
	double steering_value =  - Kp * p_error 
							- Kd * d_error
							- Ki * i_error;
/*
	if (steering_value < -1) {
		steering_value = -1;
	}
	else if (steering_value > 1) {
		steering_value = 1;
	}*/

	std::cout << "Total Error: " << TotalError() << std::endl;
	std::cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << std::endl;

	return steering_value;
}

void PID::Twiddle() {

	if (step_counter > twiddle_interval) {

		double curr_avg_cte = TotalError();
		bool go_to_next = false;

		// The code block for initial twiddling
		if (tw_best_avg_cte < 0) {
			tw_best_avg_cte = curr_avg_cte;
			tw_curr_direction = 0;
			coefficient = 0;

			// Let's pick up one out of [dp di, dd]
			while (tw_param_diffs[coefficient] == 0) {
				coefficient = (coefficient + 1) % 3;
			} 
		}

		double diff_sum = tw_param_diffs[0] + tw_param_diffs[1] + tw_param_diffs[2];
		std::cout << "Diff sum: " << diff_sum << std::endl;

		if (diff_sum > twiddle_tolerance) {

			std::cout << "Twiddling!" << std::endl;
			double increment_param = 0.0;
			switch (tw_curr_direction) {
			
			//Case for initial twiddling
			case 0:
				increment_param = tw_param_diffs[coefficient];
				tw_curr_direction = 1;
				break;

			//Case for positive direction adjustment
			case 1:
				//If it works well, let's put reward
				if (curr_avg_cte < tw_best_avg_cte) {
					tw_best_avg_cte = curr_avg_cte;
					tw_param_diffs[coefficient] *= 1.1;
					go_to_next = true;
				}
				//If it does not work, let's flip adjustment direction.
				else {
					increment_param = -2 * tw_param_diffs[coefficient];
					tw_curr_direction = -1;
				}
				break;
			//Case for negative direction adjustment
			case -1:
				//If it works well, let's put reward
				if (curr_avg_cte < tw_best_avg_cte) {
					tw_best_avg_cte = curr_avg_cte;
					tw_param_diffs[coefficient] *= 1.1;
				}
				//If not, we might be able to think that this parameter should not be changed. 
				//So let's put penalty on this. 
				else {
					increment_param = tw_param_diffs[coefficient];
					tw_param_diffs[coefficient] *= 0.9;
				}
				go_to_next = true;
			}

			//Now let changes make effect.
			switch (coefficient) {
			case 0:
				Kp += increment_param;
				break;
			case 1:
				Kd += increment_param;
				break;
			case 2:
				Ki += increment_param;
				break;
			}
			if (go_to_next) {
				tw_curr_direction = 0;
				do {
					coefficient = (coefficient + 1) % 3;
				} while (tw_param_diffs[coefficient] == 0);
			}

			//Let's clear current iteration's total erros and counters.
			//This is something like a mini batches!
			ResetTotalError();
		}
	}
}