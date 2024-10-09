#include <iostream>

#include "../tdd/tdd.cpp"

/**
 * @brief Main function to demonstrate the PIDController class.
 *
 * Intitialize an instance of PIDController, this would help in performing the
 * basic PID controller loop. It calculates the error, proportional, integral,
 * and derivative terms, and adjusts the output value accordingly. The output
 * value is restricted between the defined max and min limits.
 *
 * @return int Return status of the program (0 if successful).
 */

int main() {
  // Initialize the instance.
  PIDController pid(0.8, 0.6, 0.3, 20.0, 30.0, 0.1, 0.0, 0.0, 0.0);

  double setpoint = pid.setpoint;  // Set desired setpoint value.
  double output_value = pid.current_value;  // Current output value of the system.

  // Print the initial setpoint value.
  std::cout << "Setpoint: " << setpoint << std::endl;

  double max = 100;      // Initialize the Maximum output limit.
  double min = -100;     // Initialize the Minimum output limit.

  // Starting the PID control loop for 100 iterations.
  for (int i = 0; i <= 100; i++) {
    // Calculating the current error.
    double error = pid.compute_error(setpoint, output_value);

    // Calculating the proportional term.
    double proportional_term = pid.PropControl(error);

    // Calculating the integral term.
    double integral_term = pid.IntControl(error);

    // Calculating the derivative term.
    double derivative_term = pid.DerControl(error);

    // Calculating the final output value .
    double output_value = pid.calcOutput(error);

    // Restrict to max/min
    if (output_value > max)
      output_value = max;
    else if (output_value < min)
      output_value = min;

    // Print the current output value.
    std::cout << "Current Value: " << output_value << std::endl;
  }
}
