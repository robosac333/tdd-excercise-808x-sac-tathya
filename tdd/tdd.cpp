#include "../include/tdd.hpp"

#include <iostream>


// Implementing PID class
PIDController::PIDController(double Kp, double Ki, double Kd, double setpoint, double current_value, double dt)
    : GetterSetter(), setpoint(setpoint), current_value(current_value) { // Call to base class constructor
    this->setKp(Kp);
    this->setKi(Ki); 
    this->setKd(Kd); 
    this->setdt(dt);  // Set dt correctly
}

// Method to implement proportional control
double PIDController::PropControl(double error){
    // Implement the proportional control 
    return Kp * error;
};


// Method to implement integral control
double PIDController::IntControl(double error){

    // Implement here
};

// Method to implement derivative control
double PIDController::DerControl(double error){

    // Implement here
};

double PIDController::calcOutput(double error){
    double p_out = PropControl(error);
    double i_out = IntControl(error);
    double d_out = DerControl(error);
    // Combine and implement here and put the condition of control
};

// Method to implement derivative control
double PIDController::compute(double setpoint, double current_value){
    // Implement here
};

