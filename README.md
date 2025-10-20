# Ackermann Controller

![CICD Workflow status](https://github.com/AakashDammala/ackermann-controller/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) 
[![codecov](https://codecov.io/gh/AakashDammala/ackermann-controller/branch/main/graph/badge.svg)](https://codecov.io/gh/AakashDammala/ackermann-controller) 
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

## Overview
This project is built as part of ENPM-700 Software Development for Robotics course. Here, we develop a practical implementation of an Ackermann steering controller, incorporating unit testing and CI/CD pipelines.

The Ackermann steering controller computes wheel speeds (RPMs) and steering angles for a vehicle with Ackermann steering geometry, given desired linear and angular velocities.

Project team:
- Aakash Shetty Dammala (asd@umd.edu)
- Siddhant Deshmukh (iamsid@umd.edu)
- Dayanidhi Kandade (dotsv@umd.edu)

## Instructions to run the project

```bash
# Download the code:
git clone https://github.com/AakashDammala/ackermann-controller.git
cd ackermann-controller

# Configure the project
cmake -S ./ -B build/

# Build the project
cmake --build build/

# Run the program
./build/app/shell-app

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
```

## Project Structure
```
ackermann-controller/
├── app/                    # Main application
├── include/               # Header files
├── libs/                  # Library source files
│   ├── libackermann/     # Ackermann controller implementation
│   └── libpid/          # PID controller implementation
├── test/                 # Unit tests
└── docs/                 # Documentation (doxygen documentation, test coverage reports, uml diagrams)
```

## Features
- Ackermann steering kinematics computation
- Configurable vehicle parameters:
  - Drive width and length
  - Wheel radius
  - Steering angle limits
  - Velocity limits
- PID control for wheel speed control
- Comprehensive unit tests
- Documentation using Doxygen

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.


