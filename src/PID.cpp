#include "PID.h"
#include <iostream>
#include <limits>
#include <iomanip>

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  p_error = 0;
  i_error = 0;
  d_error = 0;
  prev_cte = 0;
  total_Error = 0;
  RMSE_Sum = 0;
  RMSE_Count = 0;
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */

  // Update main error
  p_error = cte;

  // Update integrator 
  i_error += cte;

  // Calculate derivative and update prev value
  d_error = cte - prev_cte;
  prev_cte = cte;

  // Update total error - RMSE function
  RMSE_Sum += cte*cte;
  RMSE_Count++;
  total_Error = RMSE_Sum/RMSE_Count;
}

double PID::ControlDemand()
{
   /**
   * Output PID demand 
   */

  // 
  
  // Debug step
  std::cout << std::setprecision(3) << std::fixed;
  std::cout << total_Error << "\t" << Kp * p_error << "\t" << Ki * i_error << "\t" << Kd*d_error << "\t\t";

  return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return total_Error; 
  // 
}