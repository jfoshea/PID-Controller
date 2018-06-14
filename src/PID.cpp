#include "PID.h"
#include <iostream>
#include <numeric>
using namespace std;

//==============================================================================
//  @brief  PID::PID()
//          Constructor for PID Class.
//          
//==============================================================================
PID::PID() {}

//==============================================================================
//  @brief  PID::~PID()
//          Destructor for PID Class.
//          
//==============================================================================
PID::~PID() {}

//==============================================================================
//  @brief  PID::Init()
//          Initialize the PID Controller.
//          
//  @param  double kp: Proportional control value 
//          double ki: Integral control value
//          double kd: Derivative control value
//  @return: void
//==============================================================================
void PID::Init( double Kp, double Ki, double Kd ) {

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

}

//==============================================================================
//  @brief  PID::UpdateError()
//          Update the PID error variables
//          
//  @param  double cte: Cross Track Error
//  @return: void
//==============================================================================
void PID::UpdateError( double cte ) {

  double prev_cte = p_error;

  p_error  = cte;
  i_error += cte;
  d_error  = cte - prev_cte;
}

//==============================================================================
//  @brief  PID::TotalError()
//          Update the PID error variables
//          
//  @param  None
//  @return: double total_error 
//==============================================================================
double PID::TotalError() {

  double steer_value;

  steer_value = -Kp*p_error - Ki*i_error - Kd*d_error;
  
  // limit the steer angle between 1 to -1
  if( steer_value > 1 ) {
    steer_value = 1;
  }
  else if ( steer_value < -1 ) {
    steer_value = -1;
  }
  
  return steer_value;
}

//======================================================================
//  @brief  PID::FindCoefficients()
//          Find Optimum PID coefficient (Kp, Ki, Kd)
//          
//  @param:  current cross track error
//  @return: void 
//==============================================================================
void PID::FindCoefficients( const double total_error ) {

	double *p;
	double *dp;
  int param = 0;
  auto error = total_error;
  auto best_error = 1000.0;
  dp = &Kp; 
  
  while( best_error > 0.001 )
  {
    param = ( param + 1 ) % 3;
    switch( param )
    {
      case Kp_PARAM:
	  	  p = &Kp;
        break;
      case Ki_PARAM:
	  	  p = &Ki;
        break;
      case Kd_PARAM:
	  	  p = &Kd;
        break;
      default:
        break;
    }
    
    // TODO finish this function
    if( error < best_error ) 
    {
      best_error = error;
      *dp *= 1.1;
	  }
	  else 
    {
      *p += *dp;
      *dp *= 0.9;
    }
  }  

  error = 0.0;
}

