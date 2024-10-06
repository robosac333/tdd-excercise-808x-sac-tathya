#include <gtest/gtest.h>
#include "../tdd/tdd.cpp"
#include <cmath>

class PIDControllerTest : public ::testing::Test {
protected:
    PIDController* pid;

    void SetUp() override {
        // This is called before each test
        pid = new PIDController(0.8, 0.6, 0.3, 20.0, 30.0, 0.1);
    }

    void TearDown() override {
        // This is called after each test
        delete pid;
    }
};

// Test Proportional Control
TEST_F(PIDControllerTest, ProportionalControl) {
    EXPECT_DOUBLE_EQ(pid->PropControl(-10), -8.0);
    EXPECT_DOUBLE_EQ(pid->PropControl(5), 4.0);
}

// Test Integral Control (this will fail as it's not implemented)
TEST_F(PIDControllerTest, IntegralControl) {
    EXPECT_DOUBLE_EQ(pid->IntControl(-10), 0.0);
}

// Test Derivative Control (this will fail as it's not implemented)
TEST_F(PIDControllerTest, DerivativeControl) {
    EXPECT_DOUBLE_EQ(pid->DerControl(-10), 0.0);
}

// Test Calculate Output (this will fail as it depends on unimplemented methods)
TEST_F(PIDControllerTest, CalculateOutput) {
    EXPECT_THROW(pid->calcOutput(-10), std::runtime_error);
}

// Test Compute Error
TEST_F(PIDControllerTest, ComputeError) {
    EXPECT_DOUBLE_EQ(pid->compute_error(20, 30), -10.0);
    EXPECT_DOUBLE_EQ(pid->compute_error(30, 20), 10.0);
}

// Test full PID computation (this will fail as it depends on unimplemented methods)
TEST_F(PIDControllerTest, FullPIDComputation) {
    EXPECT_THROW(pid->compute_error(20, 30), std::runtime_error);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}