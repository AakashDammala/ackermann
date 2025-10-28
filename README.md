# Ackermann Controller

![CICD Workflow status](https://github.com/AakashDammala/ackermann-controller/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) 
[![codecov](https://codecov.io/gh/AakashDammala/ackermann-controller/branch/main/graph/badge.svg)](https://codecov.io/gh/AakashDammala/ackermann-controller) 
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

This project has been built on top of [cpp-boilerplate-v2](https://github.com/TommyChangUMD/cpp-boilerplate-v2) repository of [Tommy Chang](https://github.com/TommyChangUMD)

## Project Overview & Description
This project is built as part of ENPM-700 Software Development for Robotics course. Here, we develop a practical implementation of an Ackermann steering controller, incorporating unit testing and CI/CD pipelines.

The Ackermann steering controller computes wheel speeds (RPMs) and steering angles for a vehicle with Ackermann steering geometry, given desired linear and angular velocities.

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

# Generate static code analysis output using cpp check
find app/ include/ libs/ test/ -type f \( -name "*.cpp" -o -name "*.hpp" \)     -exec cppcheck --enable=all --inconclusive --std=c++17 {} +     > result/cppcheck_output.txt 2>&1

# Generate compilation output
cmake --build build/ > result/build_output.txt 2>&1
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


