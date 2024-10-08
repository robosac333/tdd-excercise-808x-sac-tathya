#include "../include/tdd.hpp"
#include <iostream>


/**
 * @brief Construct a new PIDController object.
 * 
 * @param Kp_ Proportional gain.
 * @param Ki_ Integral gain.
 * @param Kd_ Derivative gain.
 * @param setpoint Desired target value.
 * @param current_value Current system output.
 * @param dt_ Time difference between iterations.
 * @param prev_error Previous error value (initialized to 0.0).
 * @param integral Integral term (initialized to 0.0).
 * @param derivative Derivative term (initialized to 0.0).
 */
PIDController::PIDController(double Kp_, double Ki_, double Kd_, double setpoint, double current_value, double dt_, double prev_error, double integral, double derivative)
    : Kp(Kp_), Kd(Kd_), Ki(Ki_), setpoint(setpoint), current_value(current_value), dt(dt_), prev_error(0.0), integral(0.0), derivative(0.0) { 
}

/**
 * @brief Computing the error between the setpoint and current value that would be used to compute the proportional, integral and derivative gains.
 * 
 * @param setpoint Desired target value.
 * @param current_value Current system output.
 * @return double (Difference between the setpoint and the current_value).
 */
double PIDController::compute_error(double setpoint, double current_value){
    return setpoint - current_value;
};

/**
 * @brief Calculating the final output of the PID controller.
 * 
 * To get the final output of the PID controller this method combines the proportional, integral, and derivative control terms.
 * 
 * @param error The current error value.
 * @return double (The computed final output of the PID controller).
 */
double PIDController::calcOutput(double error){
    double p_out = PropControl(error); // Getting the proportional control term.
    double i_out = IntControl(error); // Getting the integral control term.
    double d_out = DerControl(error); // Getting the derivative control term.
    
    double output = p_out + i_out + d_out;
    prev_error = error; // Saving error for the next iteration.

    return output;
};

/**
 * @brief Calculating the proportional control term.
 * 
 * @param error The current error value.
 * @return double The proportional control term output.
 */
double PIDController::PropControl(double error){
    return Kp * error;// Implementing the proportional control.
};

/**
 * @brief Calculating the integral control term.
 * 
 * @param error The current error value.
 * @return double The integral control term output.
 */
double PIDController::IntControl(double error){
    integral = integral + error * dt;
    return Ki * integral;
};

/**
 * @brief Calculating the derivative control term.
 * 
 * @param error The current error value.
 * @return double The derivative control term output.
 */
double PIDController::DerControl(double error){
    derivative = (error - prev_error) / dt; 
    return Kd * derivative;
};
