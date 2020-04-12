#include "PID.h"
#include <math.h>
#include<algorithm>

PID::PID() {
  p_error = 0;
  i_error = 0;
  d_error = 0;

  Kp = 0;
  Ki = 0;
  Kd = 0;
  i_max = 0;
  out_max = 0;
  UseAW = 0;
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_,double i_max_,double out_max_,bool UseAW_) {


  p_error = 0;
  i_error = 0;
  d_error = 0;
  last_cte = 0;

  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  i_max = std::max(0.0,i_max_);
  out_max = std::max(0.0,out_max_);
  UseAW = UseAW_;

}

double PID::CalcPID(double cte,double delta_t,int reset) {
  if (reset)
  {
    last_cte = cte;
  }

  i_error += Ki*delta_t * cte;
  i_error = std::max(-i_max,std::min(i_max,i_error));
  p_error = Kp*cte;
  d_error = Kd*(cte - last_cte)/delta_t;
  output = p_error + d_error + i_error;
  output = std::max(-out_max,std::min(out_max,output));
  last_cte = cte;
  return output;
}


void PID::UpdateParameter(double Kp_, double Ki_, double Kd_,double i_max_,double out_max_) {


  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  i_max = std::max(0.0,i_max_);
  out_max = std::max(0.0,out_max_);

}
