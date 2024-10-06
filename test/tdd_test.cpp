#include<gtest/gtest.h>

#include "../tdd/tdd.cpp"
#include <iostream>

int main(){

PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1);

    double setpoint = pid.setpoint;
    double output_value = pid.current_value;

    std::cout << "Setpoint: " << setpoint << std::endl;

    double integral = 0;
    double pre_error = 0;
    double max = 100;
    double min = -100;

    for(int i=0; i<=100; i++){
        double error = pid.compute_error(setpoint, output_value);
        
        double proportional_term = pid.PropControl(error);

        double integral_term = pid.IntControl(error);

        double derivative_term = pid.DerControl(error);

        double output_value = pid.calcOutput(error);

        // Restrict to max/min
        if( output_value > max )
            output_value = max;
        else if( output_value < min )
            output_value = min;

        pre_error = error;

        std::cout << "Current Value: " << output_value << std::endl;

}
}

TEST(tdd_test, this_should_pass) {
  EXPECT_EQ(1, 1);
}

TEST(dummy_test, this_should_pass_too) {
  EXPECT_EQ(my_function1(3), 3);
}

TEST(dummy_test, this_will_fail) {
  EXPECT_EQ(my_function2(3.2), 3.2);
}

