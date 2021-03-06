#include "PID.h"
#include <algorithm>

//Implement the pid update function, such that given the current speed
// the function returns a correct throttle values based on the PID values
//Note that you can use the following member variables to store values between update steps:
// - float d_kP : the P parameter 
// - float d_kI : the I parameter
// - float d_kD : the D parameter
// - float d_prevError : the previous error
// - float d_integral : a variable to sum the integral over
// (See PID.h for the formal difinitions)
float PID::update(float current)
{
    float error = d_target - current;
    float p = error;
    float i = d_integral + error * 0.02;
    float d = (error - d_prevError) / 0.02;
    d_prevError = error;
    d_integral = i;
    d_integral = std::max(d_integral, float(-2.0));
    d_integral = std::min(d_integral, float(2.0));
    
    return d_kP * p + d_kI * i + d_kD * d;
}
