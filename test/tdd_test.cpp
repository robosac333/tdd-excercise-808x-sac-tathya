#include <gtest/gtest.h>
#include "../include/tdd.hpp"  // Including the header file for the PIDController class
#include <cmath>

// Test Proportional Control
TEST(PIDControllerTest, ProportionalControl) {
    // Creating a PID controller instance
    PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1, 0.0, 0.0, 0.0);
    
    // Testing proportional control
    double error = -10;
    EXPECT_DOUBLE_EQ(pid.PropControl(error), -8.0);
}

// Test Integral Control
TEST(PIDControllerTest, IntegralControl) {
    PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1, 0.0, 0.0, 0.0);

    double error = -10;
    EXPECT_DOUBLE_EQ(pid.IntControl(error), -0.6);  // Expect near due to floating-point precision
}

// Test Derivative Control
TEST(PIDControllerTest, DerivativeControl) {
    PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1, 0.0, 0.0, 0.0);

    double error = -10;
    pid.compute_error(0, 0); // Initialize prev_error to 0
    EXPECT_DOUBLE_EQ(pid.DerControl(error), -30.0);  // Derivative gain Kd = 3, dt = 0.1, error change = -10
}

// Test Calculate Output
TEST(PIDControllerTest, CalculateOutput) {
    PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1, 0.0, 0.0, 0.0);

    double error = -10;
    EXPECT_DOUBLE_EQ(pid.calcOutput(error), -38.6);  // Output should be -8 + (-0.06) + (-30) = -38.606
}

// Test Compute Error
TEST(PIDControllerTest, ComputeError) {
    PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1, 0.0, 0.0, 0.0);

    double setpoint = 20;
    double current_value = 30;
    EXPECT_DOUBLE_EQ(pid.compute_error(setpoint, current_value), -10.0);
}
