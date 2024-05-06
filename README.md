# State Observers Library

## Overview

The State Observers Library provides a set of classes for implementing state observer algorithms, including Luenberger observers and Kalman filters. These observers are widely used in control systems for estimating the internal state variables of a dynamic system from measurements of its outputs.

# Contributing 

Contributions to this project are welcome! You can contribute by reporting bugs, suggesting new features, or submitting pull requests.

## TODO List

- [x] Base class of state observer, Luenberger, Kalman Filter
- [ ] Create a class to represent a dynamic system (general and linear) in state space format that contains A,B,C,D matrixes (or f,g,.. functions) with update methods, etc, and passed to state observer classes as input (not directly A,B,C,D)
- [ ] Implement EKF, UKF, Particle Filter, etc.
- [x] Test State Observer Base Class.
- [ ] Test Luenberger, Kalman Filter, and new one.
- [ ] Set Github Workflows for CI.
- [ ] Open a discussion about the necessity to convert to a ROS2 Pluginlib.
