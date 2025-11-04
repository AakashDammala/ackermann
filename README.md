# Ackermann Controller

![CICD Workflow status](https://github.com/AakashDammala/ackermann-controller/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg)
[![codecov](https://codecov.io/gh/AakashDammala/ackermann-controller/branch/main/graph/badge.svg)](https://codecov.io/gh/AakashDammala/ackermann-controller)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

This project has been built on top of [cpp-boilerplate-v2](https://github.com/TommyChangUMD/cpp-boilerplate-v2) repository of [Tommy Chang](https://github.com/TommyChangUMD)

## Project Overview & Description
This project is built as part of ENPM-700 Software Development for Robotics course. Here, we develop a practical implementation of an Ackermann steering controller, incorporating unit testing and CI/CD pipelines.

The Ackermann steering controller uses **two PID loops (velocity and steering)** to compute wheel speeds (RPMs) and steering angles for a vehicle with Ackermann steering geometry, given desired linear and angular velocities (**$v, \omega$**).

## Personnel & Roles
- Aakash Shetty Dammala (asd@umd.edu)
- Siddhant Deshmukh (iamsid@umd.edu)
- Dayanidhi Kandade (dotsv@umd.edu)

|        | **Driver** | **Navigator** | **Design Keeper** |
|--------|------------|---------------|-------------------|
| Phase0 | Akash      | Siddhant      | Dayanidhi         |
| Phase1 | Siddhant   | Dayanidhi     | Akash             |
| Phase2 | Dayanidhi  | Akash         | Siddhant          |

## Project deliverables - Phase 1
  - Project proposal pdf - [link](https://drive.google.com/drive/folders/17YOXgnPOURfj6kv4C_gz0j2qia5i_SLz)
  - Task backlog - [link](https://docs.google.com/spreadsheets/d/1MSmNYYrsdP3VOwCCngEbLwx22oEamR3j0daP1eC6vfI)
  - Quad chart - [link](https://drive.google.com/drive/folders/1tBJrR3zndVTcWkPMcNMntL4yrfq1HN4L)
  - Project proposal video - [link](https://drive.google.com/drive/folders/18yC6tSubWybAqKMQM1WBmayf3rTnOC_k)
  - Sprint planning & review notes - [link](https://docs.google.com/document/d/1Z2aq4fQkFDU_wFhV7ZzRSlVKz1qRIT2GHG1o_Dp-wCI)

## Overview of Phase 0 of the project
1. We propose a modular C++17 Ackermann Steering Controller to convert high-level planner commands ($v$, $\omega$) into actuator-level steering and wheel velocity outputs for Acme's autonomous vehicle. 
2. In Phase 0, we successfully established the complete project structure, CI/CD pipeline with CodeCov, and all unit tests required for our Test-Driven Development (TDD) approach. 
3. We are now proceeding with an Agile (AIP) process to implement the core kinematic logic.

**Operation** - setup the project scaffolding including directory structure, build system, CI pipeline, design and planning and test cases under TDD framework

**Dependencies** - cpp-boilerplate-v2 was used as the base, along with libraries such as Google test and other dependencies such as CodeCov.

**Known bugs / Potential issues** - All the test cases fail, since the core implementation will be done in the next phase of development.

## Overview of Phase 1 of the project
1. Phase 1 involved implementing the core kinematics for the ackermann library, for linear velocity and steering angle. 
2. Further processing the output of kinematic model's wheel velocity to get wheel rpms, followed by verifying all pre-written test cases locally. 
3. The code is then verified for the test coverage by the CI trigger in Github.   

**Operation** - implemented the core kinematics of ackermann library

**Dependencies** - test cases written in phase 0 for ackermann library

**Known bugs / Potential issues** - This library would be stable for lower speeds and accelerations only, as PID controller will be implemented in next phase of the development.

## Instructions to run the project

```bash
# Download the code:
git clone [https://github.com/AakashDammala/ackermann-controller.git](https://github.com/AakashDammala/ackermann-controller.git)
cd ackermann-controller

# Configure the project
cmake -S ./ -B build/

# Build the project
cmake --build build/

# Run the program (simulation demo)
# Usage: <seconds> <dt> <v_ref> <w_ref> <csv_path> [--verbose]
./build/app/shell-app 5.0 0.01 1.0 0.5 data/sim_output.csv --verbose

# Run unit tests
./build/test/cpp-test

# Generate and view the code coverage report
# Configure CMake to generate coverage report
cmake -D WANT_COVERAGE=ON -D CMAKE_BUILD_TYPE=Debug -S ./ -B build/
# Build the project and generate coverage
cmake --build build/ --clean-first --target all test_coverage
# View the coverage report
open build/test_coverage/index.html

# Generate and view doxygen documentation
cmake --build build/ --target docs
open docs/html/index.html

# Generate and view UML diagrams
# install clang-uml and plantuml
clang-uml
cd docs/diagrams
plantuml uml_class_ackermann_controller_library.puml
open uml_class_ackermann_controller_library.png

# Generate static code analysis output using cpp check
cppcheck --project=build/compile_commands.json > result/cppcheck_output.txt 2>&1

# Generate compilation output
cmake --build build/ > result/build_output.txt 2>&1
````

## Project Structure

```
ackermann-controller/
.
├── app
│   ├── CMakeLists.txt
│   └── main.cpp
├── CMakeLists.txt
├── data
│   └── sim_output.csv
├── docs
│   └── html
│   └── mainpage.dox
├── include
│   ├── libackermann.hpp
│   └── libpid.hpp
├── LICENSE
├── README.md
├── result
│   ├── build_output.txt
│   └── cppcheck_output.txt
├── src
│   ├── libackermann.cpp
│   └── libpid.cpp
├── test
│   ├── CMakeLists.txt
│   ├── main.cpp
│   └── test.cpp
└── UML-diagrams
    ├── Initial Diagram
    │   └── uml_class_ackermann_controller_library.png
    └── Revised Diagrams
        ├── Activity Diagram.jpeg
        └── Revised-Class-Diagram.png
```

## Dependencies

This project requires the following tools and libraries:
```
| Dependency       | Version | Purpose                      |
| :--------------- | :------ | :--------------------------- |
| **C++ Compiler** | C++17   | Core language                |
| **CMake**        | 3.14+   | Build system                 |
| **GoogleTest**   | 1.14.0  | Unit testing                 |
| **Doxygen**      | (Any)   | API documentation generation |
| **LCOV / GCOVR** | (Any)   | Code coverage reports        |
```
## API Reference

The project provides two main libraries: `libpid` and `libackermann`.

### `libpid` API (`PIDController` Class)

Provides a scalar PID controller with anti-windup and derivative filtering.
```
| Method                       | Description                                                                           |
| :--------------------------- | :------------------------------------------------------------------------------------ |
| `PIDController`         | Constructor: Sets gains (Kp, Ki, Kd), timestep, and output limits.                    |
| `Compute(set, meas)`         | Computes one control step. `set` is the target, `meas` is the current value.          |
| `SetAntiWindupGain(Kaw)`     | Configures the back-calculation gain for anti-windup.                                 |
| `SetIntegralLimits(min, max)`| Clamps the internal integral state to a fixed range.                                  |
| `SetDerivativeFilterTau(tau)`| Sets the time constant for the 1st-order low-pass filter on the derivative.           |
| `LastP() / LastI() / LastD()`| Returns the last computed Proportional, Integral, or Derivative term for diagnostics. |
```
### `libackermann` API

Provides the main controller logic and kinematic model.

#### Structs
```
| Struct                  | Description                                                                           |
| :---------------------- | :------------------------------------------------------------------------------------ |
| `AckermannState`        | Holds the vehicle's state: `LongitudinalSpeed`, `HeadingAngle`, `SteeringAngle`.      |
| `AckermannConfig`       | Configuration: `DriveWidth`, `DriveLength`, `WheelRadius`, limits, and deadbands.     |
| `AckermannOutput`       | Controller output: `WheelRpm[4]`, `FrontWheelSteeringAngle[2]`, and applied commands. |
| `AckermannVehicleState` | Per-wheel outputs for logging: `WheelRpm[4]`, `WheelSteeringAngle[2]`.                |
```
#### `AckermannController` Class

Converts vehicle-level references ($v$, $\omega$) into wheel commands.
```
| Method                      | Description                                                                      |
| :-------------------------- | :------------------------------------------------------------------------------- |
| `AckermannController`  | Constructor: Injects the vehicle config and two `PIDController` instances.       |
| `Compute(vRef, wRef, t)`    | The main update loop. Computes wheel RPMs from velocity and yaw rate references. |
| `Observe(vMeas, deltaMeas)` | (Optional) Feeds external measurements back into the PID loops.                  |
| `Reset()`                   | Resets internal state (e.g., last commands, time).                               |
```
#### `AckermannModel` Class

A simple kinematic "plant" model used for simulation.
```
| Method                      | Description                                                                           |
| :-------------------------- | :------------------------------------------------------------------------------------ |
| `AckermannModel`       | Constructor: Sets config, initial state, and fixed time step.                         |
| `Update(accel, steerAngle)` | Integrates the state forward by one time step using acceleration and steering inputs. |
| `GetVehicleState()`         | Computes the per-wheel RPMs and steering angles from the current state.               |
```
## License

This project is licensed under the MIT License - see the [LICENSE](https://www.google.com/search?q=LICENSE) file for details.

