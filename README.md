# Development of Swarm Robots Formation for ACME Robotics
A C++ Module for new robotics-based product of ACME Robotics using high-quality engineering practices for development of Multi-robots/ swarm actions using swarm algorithms with 20 or more robots simultaneously to arrange themselves in various geometric formations.

[![Build](https://github.com/roboticistjoseph/Kamikaze-Drones/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/roboticistjoseph/Kamikaze-Drones/actions/workflows/build_and_coveralls.yml)
[![codecov](https://codecov.io/gh/roboticistjoseph/Kamikaze-Robots/branch/master/graph/badge.svg?token=SNUGOUNGOF)](https://codecov.io/gh/roboticistjoseph/Kamikaze-Robots)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

---

## Authors
 - [Aneesh Chodisetty](https://github.com/z-Ash-z) - 117359893
 - [Bhargav Kumar Soothram](https://github.com/Bhargav-Soothram) - 117041088
 - [Joseph Pranadeer Reddy Katakam](https://github.com/roboticistjoseph) - 117517958

## Table of Contents
1. [Introduction](#introduction)
2. [Deliverables](#deliverables)
3. [Proposal Documentation](#project-proposal)
4. [Development Process](#system-design)
5. [UML Diagrams](#system-architecture)
6. [Dependencies and Tools](#dependencies)
7. [Run the Software](#how-to-build-and-run-demo)
8. [Code Coverage](#how-to-build-for-test-coverage)
9. [Unit Testing](#how-to-run-unit-tests)

## Introduction
  
  Our team will be using ‘Kamikaze Drones’ as the project code name to help protect Acme’s secret product plans. The name means ‘Divine Wind’ in Japanese and is inspired from the ‘[Kamikaze swarm operation](https://www.youtube.com/watch?v=3d28APIfwSI)’ by company ‘STM’.

  In this proposal, our team has focused on implementing one of the trending applications of swarm drones, which is to form different geometric shapes using a swarm of 20 or more drones.

  In order to achieve this, state-of-art path-planning swarm algorithms will be deployed. The performance validation of this project will be done using the Gazebo simulation depicting a real-time demo of the application. Acme can then utilize this package in its 5-year robotics-based product roadmap.


## Deliverables

  1. Proposal Documentation
  2. UML Diagrams
  3. Project Package with demonstrated OOPs concepts
  4. CI (Code Integration) using GitHub
  5. Code Coverage using Coveralls
  6. Unit Tests using Google Test Framework
  7. Developer Level Documentation
  8. Static code analysis with cppcheck
  9. Google C++ Style guide with cpplint validation

### Project proposal

  - The project proposal document can be found [here](/assets/Kamikaze_Drones_Proposal.pdf).  
  <!-- - The proposal video can be found [here](https://youtu.be/7sqIBtfbFjk).   -->
  - The quadchart can be found [here](/assets/Quadchart_phase0.pdf).  

<!-- ### Sample Output
![Sample Output](/results/sample_package_output.png)   -->

## System Design

### Development methodology

  Agile software development model will be used for the development process where tasks will be tracked using a backlog table. The software is designed in a Test-Driven Development fashion and implemented using Pair programming technique. The tasks will be outlined for every sprint and after each sprint, the roles of the pair-programming group will be interchanged.

## Development process
  - Using the Software Engineering Practices, all design and development work was followed Agile Iterative Process (AIP) where product backlog, iteration backlog, and work log were maintained usign appropriate measure. The Sheet can be accessed [here](https://docs.google.com/spreadsheets/d/1BQCqN5Qy6YmZE76j3XB-w8Y3W8QGNNPGS1bdCdOAWbg/edit#gid=0)
  - Each sprint's notes and reviews have been documented [here](https://docs.google.com/document/d/1rGsBwGwRbpP7cR_gu-ueWbK_iysQTQdyqjFzTM7u1lA/edit?usp=sharing)
  
### System architecture
- The class diagram can be found [here](/UML/revised/Class_Diagram.png).

- The flow of our system is as follows:  

![Activity Diagram](/UML/revised/Activity_Diagram.png). 


### Dependencies  

| Name | Version | License |
| :--- | :--- | :--- |
| Ubuntu | 20.04(LTS) | FSF Licenses |
| ROS 2 | Humble Hawksbill | Apache License 2.0 |
| C++ | 14 | Creative Commons Attribution-ShareAlike 3.0 Unported License |
| Cmake | 3.16.3 | BSD 3-clause License |

### Tools used  

| Usage/Type | Tool name | License |
| :--- | :--- | :--- |
| IDE | Visual Studio Code | MIT License |
| CI pipeline | Github CI | Creative Commons Attribution 4.0 |
| Code coverage | Coveralls | Coveralls, LLC |
| Running tests | Gtests | BSD 3-Clause "New" or "Revised" License |


## How to build and run Demo

### Source
```
# overlay
source /opt/ros/foxy/setup.bash
```

### Clone
```
git clone https://github.com/roboticistjoseph/Kamikaze-Robots.git
```

### Build files
```
# fresh build of packages
cd Kamikaze-Robots/
colcon build --packages-select box_bot_description
colcon build --packages-select box_bot_gazebo
colcon build --packages-select kamikaze
```

### Removing previous builds (*Optional*)
If build fails, re-run the above build commands after following the one's below.
```
cd ~/Kamikaze-Robots/
# remove any existing build files
rm -rf build/box_bot_description
rm -rf build/box_bot_gazebo
rm -rf build/kamikaze
```

### Run the Simulation
Run the below commands in two different terminals
```
# underlay
source install/local_setup.bash

# Spawn 20 Bots in gazebo
ros2 launch box_bot_gazebo multi_box_bot_launch.py
```
```
# underlay
source install/local_setup.bash

# Control the formation of spawned Bots
ros2 run kamikaze swarm_controller
```

## How to build for Test Coverage
```
cd ~/Kamikaze-Robots/
# remove any existing build files
rm -rf build/box_bot_description
rm -rf build/box_bot_gazebo
rm -rf build/kamikaze

# build with coverage
colcon build --cmake-args -DCOVERAGE=1 --packages-select box_bot_description
colcon build --cmake-args -DCOVERAGE=1 --packages-select box_bot_gazebo
colcon build --cmake-args -DCOVERAGE=1 --packages-select kamikaze

# to check log (example)
cat log/latest_build/kamikaze/stdout_stderr.log
```

### How to run Unit Tests
- In a new terminal, run the follwoing commands to Test the functionality of the cloned software.
```
cd ~/Kamikaze-Robots/
source install/setup.bash
colcon test --packages-select kamikaze
cat log/latest_test/kamikaze/stdout_stderr.log
```
Output should be:
<pre>1: [==========] Running 1 test from 1 test case.
1: [----------] Global test environment set-up.
1: [----------] 1 test from KamikazeTest
1: [ RUN      ] KamikazeTest.BasicTest
1: [WARN] [1671132175.805364740] [basic_test]: New test started.
1: First Test.
1: [       OK ] KamikazeTest.BasicTest (12 ms)
1: [----------] 1 test from KamikazeTest (12 ms total)
1: 
1: [----------] Global test environment tear-down
1: [==========] 1 test from 1 test case ran. (12 ms total)
1: [  PASSED  ] 1 test.
1: Testing Complete.
1: -- run_test.py: return code 0
1: -- run_test.py: inject classname prefix into gtest result file &apos;/home/joseph/ros2_ws2/src/Kamikaze-Robots/build/kamikaze/test_results/kamikaze/kamikaze_test.gtest.xml&apos;
1: -- run_test.py: verify result file &apos;/home/joseph/ros2_ws2/src/Kamikaze-Robots/build/kamikaze/test_results/kamikaze/kamikaze_test.gtest.xml&apos;
1/1 Test #1: kamikaze_test ....................   Passed    0.10 sec

<font color="#4E9A06">100% tests passed</font>, 0 tests failed out of 1

Label Time Summary:
gtest    =   0.10 sec*proc (1 test)

Total Test time (real) =   0.10 sec
</pre>

## Generate Coverage report
```
source install/setup.bash
ros2 run kamikaze generate_coverage_report.bash
```

## Code Analysis
Running 'cpplint' and 'cppcheck' to check for coding style and detect bugs. (Navigate to root of this cloned repo each time.)
### cpplint
Change to the root directory of the package, ```/Kamikaze-Robots```, and run:
```
cd kamikaze/
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order ./src/*.cpp ./include/kamikaze/*.hpp > ../results/cpplint.txt
```
The results of running ```cpplint``` can be found in ```/results/cpplint.txt```.

### cppcheck
Change to the root directory of the package, ```/Kamikaze-Robots```, and run:
```
cd kamikaze/
cppcheck --enable=all --std=c++17 ./src/*.cpp ./include/kamikaze/*.hpp --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > ../results/cppcheck.txt
```
The results of running ```cppcheck``` can be found in ```/results/cppcheck.txt```.
