#include "tdd.cpp"
#include <iostream>

int main(){

PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1);

double Kp = pid.getKp();
double Kd = pid.getKd();
double Ki = pid.getKi();
double dt = pid.getdt();
double setpoint = pid.setpoint;
double val = pid.current_value;

std::cout << "Setpoint: " << setpoint << std::endl;

for(int i=0; i<=100; i++){
    double error = pid.compute(setpoint, val);
    
    double proportional_term = pid.PropControl(error);

    double integral_term = pid.IntControl(error);

    double derivative_term = pid.DerControl(error);

    double val = pid.calcOutput(error)*pid.getdt();

    std::cout << "Current Value: " << val << std::endl;

}
}

