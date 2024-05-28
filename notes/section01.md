## Section 1. Introduction

First, complete the installation of [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) on your computer that runs on [Ubuntu 22.04](https://releases.ubuntu.com/22.04/).

Install Gazebo and ROS 2 Gazebo packages.

```
sudo apt install ros-humble-gazebo*
sudo apt install gazebo
```

Source the following prior to running Gazebo. Consider including this line in `.bashrc`.

```
source /usr/share/gazebo/setup.bash
```

Run Gazebo simulator.

```
gazebo
```
