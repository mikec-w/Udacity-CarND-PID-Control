#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Calculate the PID controller demand
   * @output The PID controller demand
   */
  double ControlDemand();

/** 
 * Set Feed Forward term
 * @param FF The current Feed Forward for the target
 */
  void SetFF(double FF);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  // Feed Forward
  double FF;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  // Previous CTE
  double prev_cte;

  // Total Error
  double total_Error;

  double RMSE_Sum;
  int RMSE_Count;
};

#endif  // PID_H