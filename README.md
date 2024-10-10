# tdd-excercise-808x-sac-tathya

## Part 1 Members
1. Sachin Jadhav (Driver)
2. Tathya Bhatt (Navigator)

## Part 2 
1. Mohammed Munawwar (Driver)
2. FNU Koustubh (Navigator)

# C++ Boilerplate v2 Badges
![CICD Workflow status](https://github.com/robosac333/tdd-excercise-808x-sac-tathya/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/robosac333/tdd-excercise-808x-sac-tathya/branch/main/graph/badge.svg)](https://codecov.io/gh/robosac333/tdd-excercise-808x-sac-tathya) [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

# PID Controller

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [How PID Works](#how-pid-works)
- [Tuning the PID Controller](#tuning-the-pid-controller)
- [Contributing](#contributing)
- [License](#license)

## Introduction
The PID controller adjusts the control variables by considering and adjusting the three control terms- *Proportional*, *Integral* and *Derivative*.
1. **Proportional**: Proportional control term reduces the error based on the current difference between the setpoint and the actual output.
2. **Integral**: This control term keeps track of the accumulated past errors.
3. **Derivative**: This predicts future error based on the rate of change of the error.

## PID Controller Class
The `PIDController` class has been designed to calculate the PID constants to optimize the velocity of the mobile robot based on the given target setpoint and current velocity value.
### The attributes of the class are given below: 
- `double Kp` Proportional gain
- `double Ki` Integral gain
- `double Kd` Derivative gain
- `double setpoint` Desired target value for the controlled system
- `double current_value` Current value of the controlled variable
- `double prev_error` This is to keep track of error for the next cycle
- `double integral` to calculate Ki
- `double derivative` to calculate Kd
- `double dt` Time step between updates

### The methods of the class are given as follows:
- `PIDController::PIDController`
Constructor to initialize the PID controller with the gains(`Kp`, `Ki` and `Kd`) and the other parameters that are necessary.

- `double PIDController::compute_error(double setpoint, double current_value)`
This method computes the difference between the desired setpoint and the current system output (error).

- `double PIDController::calcOutput(double error)`
This method computes the final PID output by combining proportional, integral, and derivative terms.

- `double PIDController::PropControl(double error)`
This method computes the proportional control term based on the current error.

- `double PIDController::IntControl(double error)`
This method computes the integral control term by accumulating the error over time.

- `double PIDController::DerControl(double error)`
This method computes the derivative control term based on the rate of change of the error.

## Building and running
 ```bash
 # Download the code:
  git clone https://github.com/robosac333/tdd-excercise-808x-sac-tathya.git
  cd tdd-excercise-808x-sac-tathya/
# Configure the project and generate a native build system:\changed.
  cmake -S ./ -B build/
# Compile and build the project:
# rebuild only files that are modified since the last build
  cmake --build build/
# or rebuild everything from scracth
  cmake --build build/ --clean-first
# to see verbose output, do:
  cmake --build build/ --verbose
# Run program:
  ./build/app/shell-app
# Run tests:
  cd build/; ctest; cd -
# or if you have newer cmake
  ctest --test-dir build/
 ```

## Doxygen
```bash
# Build docs:
  cmake --build build/ --target docs
# open a web browser to browse the doc
  open docs/html/index.html
# Clean
  cmake --build build/ --target clean
# Clean and start over:
  rm -rf build/
```

## To-Do
- From the history report of Github CI, it can be seen that all the tests behave according to the program design and nothing fails.
- We think that given the program designed for 2D PID Controller, there are enough tests and don't think other tests are necessary
- Further we can extend the program in the 3D coordinate system for real-world control.
