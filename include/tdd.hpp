#pragma once
#include <iostream>

/**
 * @class PIDController
 * @brief Inherits getter and setter methods from GetterSetter and implements PID control logic.
 */
class PIDController {

private:
    double Kp;
    double Ki;
    double Kd;
    double dt;

public:
    double setpoint; // Target value for the controlled system
    double current_value; // Current value of the controlled variable
    double prev_error; //Previous error for calculating derivative
    double integral; // integral 
    double derivative; // derivative

    /**
     * @brief Constructor to initialize PID controller parameters.
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param setpoint Desired target value for the controlled system
     * @param current_value Current value of the controlled variable
     * @param prev_error This is to keep track of error for the next cycle
     * @param integral to calculate Ki
     * @param derivative to calculate Kd
     * @param dt Time step between updates
     */
    PIDController(double Kp, double Ki, double Kd, double setpoint, double current_value, double dt, double prev_error, double integral, double derivative);

    /**
     * @brief Computes the proportional term of the PID control.
     * @param error The difference between setpoint and current value.
     * @return The proportional output.
     */
    double PropControl(double error);

    /**
     * @brief Computes the integral term of the PID control.
     * @param error The difference between setpoint and current value.
     * @return The integral output.
     */
    double IntControl(double error);

    /**
     * @brief Computes the derivative term of the PID control.
     * @param error The difference between setpoint and current value.
     * @return The derivative output.
     */
    double DerControl(double error);

    /**
     * @brief Computes the combined PID output.
     * @param error The difference between setpoint and current value.
     * @return The combined PID output.
     */
    double calcOutput(double error);

    /**
     * @brief Updates the PID controller for one time step and computes the output.
     * @return The computed control output.
     */
    double compute_error(double setpoint, double val);
};
