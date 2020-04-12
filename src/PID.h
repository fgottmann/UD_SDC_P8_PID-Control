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
  void Init(double Kp_, double Ki_, double Kd_,double i_max,double out_max,bool UseAW);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  double CalcPID(double cte,double delta_t,int reset);

  void UpdateParameter(double Kp_, double Ki_, double Kd_,double i_max,double out_max);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double last_cte;
  double output;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  double i_max;
  double out_max;
  bool UseAW;
};

#endif  // PID_H
