# Development of Swarm Drones
A C++ Module for new robotics-based product of ACME Robotics using high-quality engineering practices for development of Multi-robots/ swarm actions using swarm algorithms with 20 or more drones simultaneously

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
6. [Dependencies](#dependencies)
7. [Tools](#tools-used)
<!-- 3. Package hierarchy -->

## Introduction
  
  An increased usage of Unmanned Aerial and Surface Vehicles (UAVs and USVs) has been witnessed for various autonomous aerial applications in recent years. A swarm of these unmanned autonomous vehicles can perform safe collision-free navigation by adjusting the positions of drones using controlled localization and formation. In this proposal, our team has focused on implementing one of the trending applications of swarm drones, which is to form different geometric shapes using a swarm of 20 or more drones.

  Simply put, a bunch of drones form geometric shapes such as a circle or rectangle midair autonomously without any collisions amongst themselves. In order to achieve this, state-of-art path-planning swarm algorithms will be deployed. The performance validation of this project will be done using the Gazebo simulation depicting a real-time demo of the application. Acme can then utilize this package in its 5-year robotics-based product roadmap.


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

  - The project proposal document can be found [here](/assets/Swarm_Drones_Proposal.pdf).  
  - The proposal video can be found [here](https://youtu.be/7sqIBtfbFjk).  
  - The quadchart can be found [here](/assets/Quadchart_phase_0.pdf).  

<!-- ### Sample Output
![Sample Output](/results/sample_package_output.png)   -->

## System Design

### Development methodology

  Agile software development model will be used for the development process where tasks will be tracked using a backlog table. The software is designed in a Test-Driven Development fashion and implemented using Pair programming technique. The tasks will be outlined for every sprint and after each sprint, the roles of the pair-programming group will be interchanged.

## Development process
  - Using the Software Engineering Practices, all design and development work was followed Agile Iterative Process (AIP) where product backlog, iteration backlog, and work log were maintained usign appropriate measure. The Sheet can be accessed [here](https://docs.google.com/spreadsheets/d/1fHZmI5XlFYrR_24ZgUXfHm7kWQuyqo2a178b-_H9M2g/edit?usp=sharing)
  - Each sprint's notes and reviews have been documented [here](https://docs.google.com/document/d/1rGsBwGwRbpP7cR_gu-ueWbK_iysQTQdyqjFzTM7u1lA/edit?usp=sharing)
  
### System architecture
- The class diagram can be found [here](/UML/development_phase_0/ClassDiagram.png).

- The flow of our system is as follows:  

![Activity Diagram](/UML/development_phase_0/ActivityDiagram.png)  


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


## Other Tools
* ```ament_cmake```
* ```rclcpp```
* ```std_msgs```
