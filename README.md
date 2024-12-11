# State Observers Library 🚀

[![GitHub Action Status](https://github.com/JRL-CARI-CNR-UNIBS/state_observers/workflows/main/badge.svg)](https://github.com/JRL-CARI-CNR-UNIBS/state_observers)
[![codecov](https://codecov.io/gh/JRL-CARI-CNR-UNIBS/state_observers/graph/badge.svg?token=WTBYK3VQAT)](https://codecov.io/gh/JRL-CARI-CNR-UNIBS/state_observers)
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FJRL-CARI-CNR-UNIBS%2Fstate_observers.svg?type=shield&issueType=license)](https://app.fossa.com/projects/git%2Bgithub.com%2FJRL-CARI-CNR-UNIBS%2Fstate_observers?ref=badge_shield&issueType=license)

## Overview

The **State Observers Library** provides a set of classes for implementing state observer algorithms, including **Luenberger observers** and **Kalman filters**. These observers are widely used in control systems for estimating the internal state variables of a dynamic system from measurements of its outputs.

The library is designed to be modular and extensible, with a base `StateObserver` class that can be extended to implement various state estimation algorithms. It also integrates with ROS 2 through pluginlib, allowing for dynamic loading and configuration of observers and their parameters. See the [DOCUMENTATION](https://jrl-cari-cnr-unibs.github.io/state_observers/).

## Features ✨

- **Modular Design**: A base `StateObserver` class provides common functionality, making it easy to implement new observers.
- **Luenberger Observer**: Implements a Luenberger observer for linear systems.
- **Kalman Filter**: Implements a discrete-time Kalman Filter for linear systems.
- **ROS 2 Pluginlib Integration**: Observers and their parameter classes are available as ROS 2 plugins.
- **Parameter Classes**: Separate parameter classes manage configuration parameters for observers.

## Classes and Plugins

### Observer Classes

- **`StateObserver`**: Abstract base class providing the interface and common functionality for state observers.

- **`Luenberger`**: Inherits from `StateObserver`. Implements the Luenberger observer algorithm for linear systems.

- **`KalmanFilter`**: Inherits from `StateObserver`. Implements the Kalman Filter algorithm for linear systems.

### Parameter Classes

- **`StateObserverParam`**: Base class for managing common parameters, such as state-space matrices and the initial state vector.

- **`LuenbergerParam`**: Inherits from `StateObserverParam`. Adds the observer gain matrix `L` specific to the Luenberger observer.

- **`KalmanFilterParam`**: Inherits from `StateObserverParam`. Adds process noise covariance `Q`, measurement noise covariance `R`, and initial error covariance `P0` specific to the Kalman Filter.

### ROS 2 Plugins

The library provides ROS 2 pluginlib plugins for dynamic loading:

- **Parameter Plugins** (`state_observers_params_plugins.xml`):

  ```xml
  <library path="state_observers">
    <class type="state_observer::LuenbergerParam" base_class_type="state_observer::StateObserverParam">
      <description>This is a plugin for Luenberger Param ROS2 Loader</description>
    </class>
    <class type="state_observer::KalmanFilterParam" base_class_type="state_observer::StateObserverParam">
      <description>This is a plugin for Kalman Filter Param ROS2 Loader</description>
    </class>
  </library>
  ```

## Installation

To use the State Observers Library in your ROS 2 workspace, clone the repository and build it using `colcon`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/JRL-CARI-CNR-UNIBS/state_observers.git
cd ~/ros2_ws
colcon build
```

## Contributing 🤝

Contributions to this project are welcome! You can contribute by reporting bugs, suggesting new features, or submitting pull requests.

### How to Contribute

- **Report Bugs**: Use the [issue tracker](https://github.com/JRL-CARI-CNR-UNIBS/state_observers/issues) to report bugs.
- **Suggest Features**: Propose new features or enhancements.
- **Submit Pull Requests**: Fork the repository and submit pull requests for your contributions.

## TODO List

- [x] Base class of state observer, Luenberger, Kalman Filter
- [ ] Create a class to represent a dynamic system (general and linear) in state-space format that contains `A`, `B`, `C`, `D` matrices (or `f`, `g`, etc. functions) with update methods, and pass it to state observer classes as input (not directly `A`, `B`, `C`, `D`)
- [ ] Implement Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), Particle Filter, etc.
- [x] Test State Observer Base Class
- [x] Test Luenberger, Kalman Filter
- [x] Set up GitHub Workflows for Continuous Integration (CI)
- [x] Implement ROS 2 Pluginlib support for `StateObserver`, `Luenberger`, `KalmanFilter`, and their parameter classes

## License

This project is licensed under the Apache License, Version 2.0. See the [LICENSE](LICENSE) file for details.
